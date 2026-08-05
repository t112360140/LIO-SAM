#include <chrono>
#include <atomic>
#include <thread>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/string.hpp>

#include "lib/httplib.h"
#include "lib/json.hpp"
#include "lib/sys_info.h"
#include <mutex>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "lio_sam/imageProjection.cpp"
#include "lio_sam/featureExtraction.cpp"
#include "lio_sam/imuPreintegration.cpp"
#include "lio_sam/mapOptmization.cpp"
#include "point_colorizer/pointColorization.cpp"
#include "imu_tools/complementary_filter.h"

#include "lib/safeQueue.cpp"

struct ParsedBagMessage {
    std::string topic_name;
    double time_stamp;
    
    sensor_msgs::msg::Imu::SharedPtr imu_msg;
    sensor_msgs::msg::PointCloud2::SharedPtr pc_msg;
    tf2_msgs::msg::TFMessage::SharedPtr tf_msg;
    std_msgs::msg::Header::SharedPtr header_msg;
};

class OfflineRunner : public ParamServer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ~OfflineRunner() {
        if (http_server_.is_running()) {
            http_server_.stop();
        }
        if (http_thread_.joinable()) {
            http_thread_.join();
        }
    }

    std::vector<std::shared_ptr<ImageProjection>> IPs;
    std::vector<std::shared_ptr<FeatureExtraction>> FEs;
    std::shared_ptr<PointColorization> PC;
    std::shared_ptr<IMUPreintegration> IMUP;
    std::shared_ptr<TransformFusion> TF;
    std::shared_ptr<mapOptimization> MO;
    
    std::atomic<bool> is_loop_closing{false};
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubDebugStatus;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_static_;

    OfflineRunner(const rclcpp::NodeOptions &options, const std::string &bag_path) 
        : ParamServer("lio_sam_offline_runner", options) 
    {
        this->declare_parameter("pointCloudTopics", std::vector<std::string>());
        this->get_parameter("pointCloudTopics", pointCloudTopics);
        this->declare_parameter("lidarFrames", std::vector<std::string>());
        this->get_parameter("lidarFrames", lidarFrames);
        this->declare_parameter("lidarSyncDelayWindow", 0.01);
        this->get_parameter("lidarSyncDelayWindow", lidarSyncDelayWindow);

        if (pointCloudTopics.size() != lidarFrames.size() || pointCloudTopics.empty()) {
            RCLCPP_ERROR(this->get_logger(), 
                "\033[1;31m[CONFIG ERROR]\033[0m 光達 Topic 數量 (%zu) 與 Frame 數量 (%zu) 不一致或為空！", 
                pointCloudTopics.size(), lidarFrames.size());
            rclcpp::shutdown();
            return;
        }
        num_lidars = pointCloudTopics.size();
        RCLCPP_INFO(this->get_logger(), "成功偵測到 %d 顆光達配置，開始初始化...", num_lidars);

        pubDebugStatus = create_publisher<std_msgs::msg::String>("lio_sam/deebug/status", 1);

        this->declare_parameter("publishBagTF", true);
        this->get_parameter("publishBagTF", publishBagTF);
        pub_tf_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100);
        pub_tf_static_ = this->create_publisher<tf2_msgs::msg::TFMessage>(
            "/tf_static", 
            rclcpp::QoS(rclcpp::KeepAll()).transient_local()
        );
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        std::vector<std::string> static_tfs_strs;
        this->declare_parameter("static_tfs", std::vector<std::string>());
        this->get_parameter("static_tfs", static_tfs_strs);

        tf2_msgs::msg::TFMessage tf_static_msg;

        for (const auto& tf_str : static_tfs_strs) {
            std::stringstream ss(tf_str);
            std::string item;
            std::vector<std::string> tokens;
            
            while (std::getline(ss, item, ',')) {
                item.erase(0, item.find_first_not_of(" \t"));
                item.erase(item.find_last_not_of(" \t") + 1);
                tokens.push_back(item);
            }

            if (tokens.size() == 8) {
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = rclcpp::Time(0); 
                t.header.frame_id = tokens[0]; // parent
                t.child_frame_id = tokens[1];  // child

                t.transform.translation.x = std::stod(tokens[2]);
                t.transform.translation.y = std::stod(tokens[3]);
                t.transform.translation.z = std::stod(tokens[4]);

                tf2::Quaternion q;
                q.setRPY(std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]));
                t.transform.rotation.x = q.x();
                t.transform.rotation.y = q.y();
                t.transform.rotation.z = q.z();
                t.transform.rotation.w = q.w();

                tf_buffer_->setTransform(t, "offline_runner_static", true);
                
                tf_static_msg.transforms.push_back(t);
                
                RCLCPP_INFO(this->get_logger(), "載入靜態 TF: %s -> %s", tokens[0].c_str(), tokens[1].c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "靜態 TF 參數格式錯誤，應為 8 個數值: %s", tf_str.c_str());
            }
        }
        if (!tf_static_msg.transforms.empty()) {
            pub_tf_static_->publish(tf_static_msg);
        }

        IPs.resize(num_lidars);
        FEs.resize(num_lidars);
        cloudsBuffer.resize(num_lidars);
        for (int i = 0; i < num_lidars; i++) {
            IPs[i] = std::make_shared<ImageProjection>(options);
            FEs[i] = std::make_shared<FeatureExtraction>(options);

            IPs[i]->pointCloudTopic = pointCloudTopics[i];
            IPs[i]->lidarFrame = lidarFrames[i];
            FEs[i]->lidarFrame = lidarFrames[i]; 

            IPs[i]->setCloudInfoCallback([this, i](const lio_sam::msg::CloudInfo::SharedPtr msg) { 
                FEs[i]->laserCloudInfoHandler(msg); 
            });

            FEs[i]->setCloudInfoCallback([this, i](const lio_sam::msg::CloudInfo::SharedPtr msg) {
                this->syncAndMergeHandler(msg, i);
            });
        }

        
        PC = std::make_shared<PointColorization>(options);
        PC->tf_buffer_ = this->tf_buffer_;

        IMUP = std::make_shared<IMUPreintegration>(options);

        TF = std::make_shared<TransformFusion>(options);
        TF->lidarFrame = baselinkFrame;
        TF->tf_buffer_ = this->tf_buffer_;

        MO = std::make_shared<mapOptimization>(options);
        MO->tf_buffer_ = this->tf_buffer_;
        
        int max_feature_size = N_SCAN * Horizon_SCAN * num_lidars;
        MO->laserCloudOriCornerVec.resize(max_feature_size);
        MO->coeffSelCornerVec.resize(max_feature_size);
        MO->laserCloudOriCornerFlag.resize(max_feature_size, false);
        MO->laserCloudOriSurfVec.resize(max_feature_size);
        MO->coeffSelSurfVec.resize(max_feature_size);
        MO->laserCloudOriSurfFlag.resize(max_feature_size, false);


        // 綁定資料流與神經反饋
        PC->setCloudInfoCallback([this](const lio_sam::msg::CloudInfo::SharedPtr msg) { MO->laserCloudInfoHandler(msg); });
        MO->setOdomIncrementalCallback([this](const nav_msgs::msg::Odometry::SharedPtr msg) { IMUP->odometryHandler(msg); });
        MO->setOdomGlobalCallback([this](const nav_msgs::msg::Odometry::SharedPtr msg) { TF->lidarOdometryHandler(msg); });
        IMUP->setImuOdomCallback([this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            for (int i = 0; i < num_lidars; i++) IPs[i]->odometryHandler(msg);
            TF->imuOdometryHandler(msg);
        });
    
        skipLidarTopic = this->declare_parameter<double>("skipLidarTopic", 0.0);

        use_complementary_filter_ = this->declare_parameter<bool>("use_complementary_filter", false);

        if (use_complementary_filter_) {
            double gain_acc = this->declare_parameter<double>("gain_acc", 0.01);
            double gain_mag = this->declare_parameter<double>("gain_mag", 0.01);
            bool do_bias_estimation = this->declare_parameter<bool>("do_bias_estimation", true);
            double bias_alpha = this->declare_parameter<double>("bias_alpha", 0.01);
            bool do_adaptive_gain = this->declare_parameter<bool>("do_adaptive_gain", true);
            // bool use_mag = this->declare_parameter<bool>("use_mag", false);

            filter_.setDoBiasEstimation(do_bias_estimation);
            filter_.setDoAdaptiveGain(do_adaptive_gain);
            if (!filter_.setGainAcc(gain_acc)) {
                RCLCPP_WARN(this->get_logger(), "Invalid gain_acc passed to ComplementaryFilter.");
            }
            if (!filter_.setBiasAlpha(bias_alpha)) {
                RCLCPP_WARN(this->get_logger(), "Invalid bias_alpha passed to ComplementaryFilter.");
            }
            // if (!filter_.setBiasAlpha(gain_mag)) {
            //     RCLCPP_WARN(this->get_logger(), "Invalid gain_mag passed to ComplementaryFilter.");
            // }
        }

        updateExtrinsics();

        SysInfo::InitCpuUsage();
    }
    
    void start_http_server() {
        if(!enable_web) return;
        http_thread_ = std::thread([this]() {
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory(PROJECT_NAME);
            std::string web_dir = pkg_share_dir + "/web";
            std::string index_path = web_dir + "/index.html";

            try {
                if (std::filesystem::exists(index_path) && std::filesystem::is_symlink(index_path)) {
                    std::filesystem::path real_index_path = std::filesystem::canonical(index_path);
                    web_dir = real_index_path.parent_path().string();
                    std::cout << "Target full path resolved from file: " << web_dir << "\n";
                }
                else if (std::filesystem::is_symlink(web_dir)) {
                    web_dir = std::filesystem::canonical(web_dir).string(); 
                    std::cout << "Target full path: " << web_dir << "\n";
                } else {
                    std::cout << "Path: " << web_dir << " is not a symbolic link.\n";
                }
            } catch (const std::filesystem::filesystem_error& e) {
                std::cerr << "Error: " << e.what() << "\n";
            }

            http_server_.set_mount_point("/",  web_dir);

            http_server_.Get("/status", [this](const httplib::Request &, httplib::Response &res) {
                std::lock_guard<std::mutex> lock(this->status_mtx);

                double cpu_usage = SysInfo::GetCpuUsage();
                SysInfo::RamUsage ram_usage = SysInfo::GetRamUsage();

                nlohmann::json json = {
                    {"progress", this->current_progress.load()},
                    {"current", this->current_msg_count.load()},
                    {"total", this->total_msg_count.load()},
                    {"status", this->current_status.c_str()},
                    {"processing", this->is_processing_.load()},
                    {"cpu", cpu_usage},
                    {"ram", ram_usage.usagePercent},
                    {"ram_used", ram_usage.usedBytes},
                    {"ram_total", ram_usage.totalBytes}
                };
                res.set_content(json.dump(), "application/json");
            });

            http_server_.Get("/save", [this](const httplib::Request &, httplib::Response &res) {
                if (this->is_processing_.load()) {
                    this->save_requested_.store(true);
                    res.set_content("{\"status\": \"saving_queued\"}", "application/json");
                } else {
                    this->MO->saveMap();
                    res.set_content("{\"status\": \"saved_directly\"}", "application/json");
                }
            });

            http_server_.Get("/path", [this](const httplib::Request &, httplib::Response &res) {
                std::vector<float> path_data = this->MO->globalPathToArray(); 
                
                res.set_content(
                    reinterpret_cast<const char*>(path_data.data()), 
                    path_data.size() * sizeof(float), 
                    "application/octet-stream"
                );
            });

            http_server_.Get("/loop_edges", [this](const httplib::Request &, httplib::Response &res) {
                std::vector<float> edges_data = this->MO->getLoopEdgesToArray();
                
                res.set_content(
                    reinterpret_cast<const char*>(edges_data.data()), 
                    edges_data.size() * sizeof(float), 
                    "application/octet-stream"
                );
            });

            http_server_.Get("/loop_indices", [this](const httplib::Request &, httplib::Response &res) {
                std::vector<uint32_t> indices_data = this->MO->getLoopIndicesToArray();
                
                res.set_content(
                    reinterpret_cast<const char*>(indices_data.data()), 
                    indices_data.size() * sizeof(uint32_t),
                    "application/octet-stream"
                );
            });

            http_server_.Get("/map_preview", [this](const httplib::Request &req, httplib::Response &res) {
                bool color_req = true;
                if (req.has_param("color"))
                    color_req = (req.get_param_value("color") == "true");
                
                auto cloud_data = this->MO->globalMapToArray(color_req);
                
                res.set_content(
                    reinterpret_cast<const char*>(cloud_data.data()), 
                    cloud_data.size() * sizeof(cloud_data), 
                    "application/octet-stream"
                );
            });

            RCLCPP_INFO(this->get_logger(), "🌐 Web 監控面板已啟動: http://0.0.0.0:8080");
            http_server_.listen("0.0.0.0", 8080);
        });
    }

    void run(const std::string &bag_path) {
        is_processing_.store(true);
        processBag(bag_path);
        MO->publishGlobalMap();
        is_processing_.store(false);
        updateStatus("[INFO] 處理完成!");
    }

private:
    httplib::Server http_server_;
    std::thread http_thread_;
    
    std::atomic<bool> save_requested_{false};
    std::atomic<bool> is_processing_{true};

    std::atomic<uint64_t> current_msg_count{0};
    std::atomic<uint64_t> total_msg_count{0};
    std::atomic<double> current_progress{0.0};
    std::string current_status = "[INFO] 等待開始...";
    std::mutex status_mtx;

    double skipLidarTopic;

    bool publishBagTF;

    std::vector<std::string> pointCloudTopics;
    std::vector<std::string> lidarFrames;
    float lidarSyncDelayWindow;
    int num_lidars = 0;

    bool use_complementary_filter_ = false;
    imu_tools::ComplementaryFilter filter_;

    double last_imu_time_ = 0.0;
    bool filter_initialized_ = false;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    SafeQueue<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> bag_queue_{200};

    void publishStatus(string status) {
        if (pubDebugStatus->get_subscription_count() == 0)
            return;

        std_msgs::msg::String str_msg;
        str_msg.data = status;
        pubDebugStatus->publish(str_msg);
    }

    void updateStatus(std::string status) {
        std::lock_guard<std::mutex> lock(status_mtx);
        current_status = status;
        publishStatus(status);
    }

    void updateExtrinsics() {
        for (int i = 0; i < num_lidars; i++) {
            try {
                // 取得最新 base_link 到 lidar_i 的 TF
                geometry_msgs::msg::TransformStamped t = 
                    tf_buffer_->lookupTransform(baselinkFrame, lidarFrames[i], tf2::TimePointZero);
                
                // 提取旋轉矩陣
                Eigen::Quaterniond q_base_to_lidar(
                    t.transform.rotation.w, t.transform.rotation.x, 
                    t.transform.rotation.y, t.transform.rotation.z);
                Eigen::Matrix3d R_base_to_lidar = q_base_to_lidar.toRotationMatrix();
                
                // 求反矩陣 (R_lidar_to_base)
                Eigen::Matrix3d R_lidar_to_base = R_base_to_lidar.transpose();

                // 計算 IMU -> lidar_i 的旋轉矩陣，並直接覆寫 IP 內部的變數！
                // (this->extRot 維持是 IMU -> base_link 的關係)
                IPs[i]->extRot = R_lidar_to_base * this->extRot;
                IPs[i]->extRPY = R_lidar_to_base * this->extRPY;
                IPs[i]->extQRPY = Eigen::Quaterniond(IPs[i]->extRPY).normalized();

            } catch (const tf2::TransformException & ex) {

            }
        }
    }

    void processImuMsg(std::shared_ptr<sensor_msgs::msg::Imu> imu_msg) {
        if (use_complementary_filter_) {
            double current_imu_time = rclcpp::Time(imu_msg->header.stamp).seconds();
            
            if (!filter_initialized_) {
                last_imu_time_ = current_imu_time;
                filter_initialized_ = true;

                double q0, q1, q2, q3;
                filter_.update(imu_msg->linear_acceleration.x, 
                            imu_msg->linear_acceleration.y, 
                            imu_msg->linear_acceleration.z, 
                            0, 0, 0, 0);
            } else {
                double dt = current_imu_time - last_imu_time_;
                last_imu_time_ = current_imu_time;

                if (dt > 0.0) {
                    const auto &a = imu_msg->linear_acceleration;
                    const auto &w = imu_msg->angular_velocity;
                    
                    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);
                    
                    double q0, q1, q2, q3;
                    filter_.getOrientation(q0, q1, q2, q3);
                    
                    imu_msg->orientation.x = q1;
                    imu_msg->orientation.y = q2;
                    imu_msg->orientation.z = q3;
                    imu_msg->orientation.w = q0;

                    if (filter_.getDoBiasEstimation()) {
                        imu_msg->angular_velocity.x -= filter_.getAngularVelocityBiasX();
                        imu_msg->angular_velocity.y -= filter_.getAngularVelocityBiasY();
                        imu_msg->angular_velocity.z -= filter_.getAngularVelocityBiasZ();
                    }
                }
            }
        }
    }

    bool lidarSync = false;
    double first_lidar_time_ = -1.0;
    std::vector<lio_sam::msg::CloudInfo::SharedPtr> cloudsBuffer;
    void syncAndMergeHandler(const lio_sam::msg::CloudInfo::SharedPtr msgIn, int lidar_idx) {
        double current_time = msgIn->header.stamp.sec + msgIn->header.stamp.nanosec * 1e-9;

        if (first_lidar_time_ > 0 && (current_time - first_lidar_time_ > lidarSyncDelayWindow)) {
            RCLCPP_WARN(this->get_logger(), "光達同步超時！強制合併並輸出殘缺幀。");
            executeMerge();
            first_lidar_time_ = -1.0;
        }

        cloudsBuffer[lidar_idx] = msgIn;
        if (first_lidar_time_ < 0) {
            first_lidar_time_ = current_time;
        }

        bool all_filled = true;
        for (int i = 0; i < num_lidars; i++) {
            if (cloudsBuffer[i] == nullptr) {
                all_filled = false;
                break;
            }
        }

        if (all_filled) {
            executeMerge();
            first_lidar_time_ = -1.0;
        }
    }

    void executeMerge() {
        lio_sam::msg::CloudInfo::SharedPtr base_cloud = nullptr;
        for (int i = 0; i < num_lidars; i++) {
            if (cloudsBuffer[i] != nullptr) {
                base_cloud = cloudsBuffer[i];
                break;
            }
        }
        if (!base_cloud) return;

        auto mergeMsg = std::make_shared<lio_sam::msg::CloudInfo>(*base_cloud);
        mergeMsg->header.frame_id = baselinkFrame;

        pcl::PointCloud<PointType>::Ptr cornerMerge(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfMerge(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr deskewedMerge(new pcl::PointCloud<PointType>());

        #pragma omp parallel for num_threads(num_lidars)
        for (int i = 0; i < num_lidars; i++) {
            if (cloudsBuffer[i] == nullptr) continue;

            pcl::PointCloud<PointType>::Ptr tempCorner(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr tempSurf(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr tempDeskewed(new pcl::PointCloud<PointType>());
            
            pcl::fromROSMsg(cloudsBuffer[i]->cloud_corner, *tempCorner);
            pcl::fromROSMsg(cloudsBuffer[i]->cloud_surface, *tempSurf);
            pcl::fromROSMsg(cloudsBuffer[i]->cloud_deskewed, *tempDeskewed);

            try {
                geometry_msgs::msg::TransformStamped transform = 
                    tf_buffer_->lookupTransform(baselinkFrame, lidarFrames[i], tf2::TimePointZero);
                Eigen::Affine3d T_lidar_to_base = tf2::transformToEigen(transform);
                Eigen::Matrix4f T = T_lidar_to_base.matrix().cast<float>();

                pcl::transformPointCloud(*tempCorner, *tempCorner, T);
                pcl::transformPointCloud(*tempSurf, *tempSurf, T);
                pcl::transformPointCloud(*tempDeskewed, *tempDeskewed, T);

                #pragma omp critical
                {
                    *cornerMerge += *tempCorner;
                    *surfMerge += *tempSurf;
                    *deskewedMerge += *tempDeskewed;
                }
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "找不到 TF: %s", ex.what());
            }

            cloudsBuffer[i] = nullptr;
        }

        pcl::toROSMsg(*cornerMerge, mergeMsg->cloud_corner);
        pcl::toROSMsg(*surfMerge, mergeMsg->cloud_surface);
        pcl::toROSMsg(*deskewedMerge, mergeMsg->cloud_deskewed);

        mergeMsg->cloud_corner.header = mergeMsg->header;
        mergeMsg->cloud_surface.header = mergeMsg->header;
        mergeMsg->cloud_deskewed.header = mergeMsg->header;

        PC->cloudInfoHandler(mergeMsg);
        lidarSync = true;
    }

    void processBag(const std::string &bag_path) {
        rosbag2_cpp::Reader reader;
        reader.open(bag_path);

        auto metadata = reader.get_metadata();
        uint64_t total_messages = metadata.message_count;
        uint64_t processed_messages = 0;
        uint64_t first_msg_time = 0;

        if (total_messages <= 0)  {
            RCLCPP_ERROR(this->get_logger(), "\033[1;31m[BAG ERROR]\033[0m MSG 數量不足! 停止運行");
            updateStatus("[ERROR] MSG 數量不足! 停止運行");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "開始全速處理 Bag: %s", bag_path.c_str());
        updateStatus("[INFO] 開始處理!");
        auto start_wall_time = std::chrono::steady_clock::now();

        // ==========================================
        // 專門負責解碼與 SLAM 運算
        // ==========================================
        std::thread compute_thread([&]() {
            double last_print_time = 0.0;
            double last_loop_closure_time = 0.0;

            rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
            rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_ser;
            rclcpp::Serialization<std_msgs::msg::Header> header_ser;
            rclcpp::Serialization<sensor_msgs::msg::Image> img_ser;
            rclcpp::Serialization<sensor_msgs::msg::CompressedImage> comp_img_ser;
            rclcpp::Serialization<sensor_msgs::msg::CameraInfo> info_ser;
            rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_ser;

            while (rclcpp::ok()) {
                std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message;
                
                // 從佇列拿出資料。若佇列空了，會在這裡睡覺等待
                bag_queue_.pop(bag_message);

                // 如果拿到 nullptr，代表生產者讀完檔案了，結束迴圈
                if (bag_message == nullptr) {
                    break;
                }

                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                double current_bag_time = bag_message->time_stamp / 1e9;

                if (first_msg_time == 0) {
                    first_msg_time = bag_message->time_stamp;
                    last_loop_closure_time = current_bag_time;
                }

                processed_messages++;
                current_msg_count = processed_messages;
                total_msg_count = total_messages;
                double progress = total_messages > 0 ? (double)processed_messages / total_messages * 100.0 : 0.0;
                current_progress = progress;

                if (processed_messages % 500 == 0 || processed_messages == total_messages) {
                    auto current_wall_time = std::chrono::steady_clock::now();
                    double wall_elapsed = std::chrono::duration<double>(current_wall_time - start_wall_time).count();
                    double bag_elapsed = (bag_message->time_stamp - first_msg_time) / 1e9; 
                    double rtf = wall_elapsed > 0.0 ? bag_elapsed / wall_elapsed : 0.0;
                    
                    if (wall_elapsed - last_print_time >= logInterval || processed_messages == total_messages) {
                        last_print_time = wall_elapsed;
                        RCLCPP_INFO(this->get_logger(), "\n[處理進度] %.1f%% (%lu/%lu) | 現實耗時: %.1fs | Bag推進: %.1fs | 倍率: %.2f x", 
                                    progress, processed_messages, total_messages, wall_elapsed, bag_elapsed, rtf);
                    }
                    
                    char buffer[256];
                    std::snprintf(buffer, sizeof(buffer), 
                                    "[INFO] 進度: %lu/%lu\n耗時: %.1fs / %.1fs (bag/實際)\n倍率: %.2f x", 
                                    processed_messages, total_messages, bag_elapsed, wall_elapsed, rtf);
                    updateStatus(std::string(buffer));
                }

                if (save_requested_.load()) {
                    updateStatus("[INFO] 正在儲存地圖...");
                    MO->saveMap();
                    save_requested_.store(false);
                    updateStatus("[INFO] 存檔完成");
                }


                if (bag_message->topic_name == "/tf" || bag_message->topic_name == "/tf_static") {
                    auto tf_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
                    tf_ser.deserialize_message(&serialized_msg, tf_msg.get());
                    bool is_static = (bag_message->topic_name == "/tf_static");
                    for (const auto& transform : tf_msg->transforms) {
                        tf_buffer_->setTransform(transform, "offline_runner", is_static);
                    }

                    updateExtrinsics();

                    if (!publishBagTF) continue;
                    if (is_static)
                        pub_tf_static_->publish(*tf_msg);
                    else
                        pub_tf_->publish(*tf_msg);

                    continue;
                }
                
                if (enableColorization) {
                    auto img_it = std::find(imageTopics.begin(), imageTopics.end(), bag_message->topic_name);
                    if (img_it != imageTopics.end()) {
                        int cam_idx = std::distance(imageTopics.begin(), img_it);
                        if (useCompressedImages) {
                            auto img_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
                            comp_img_ser.deserialize_message(&serialized_msg, img_msg.get());
                            PC->compressedImageHandler(img_msg, cam_idx);
                        } else {
                            auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
                            img_ser.deserialize_message(&serialized_msg, img_msg.get());
                            PC->imageHandler(img_msg, cam_idx);
                        }
                    }
                    else {
                        auto info_it = std::find(cameraInfoTopics.begin(), cameraInfoTopics.end(), bag_message->topic_name);
                        if (info_it != cameraInfoTopics.end()) {
                            int cam_idx = std::distance(cameraInfoTopics.begin(), info_it);
                            auto info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
                            info_ser.deserialize_message(&serialized_msg, info_msg.get());
                            PC->cameraInfoHandler(info_msg, cam_idx);
                        }
                    }
                }

                if (bag_message->topic_name == imuTopic) {
                    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
                    imu_ser.deserialize_message(&serialized_msg, imu_msg.get());
                    processImuMsg(imu_msg);
                    
                    IMUP->imuHandler(imu_msg);
                    for(int i=0; i<num_lidars; i++) IPs[i]->imuHandler(imu_msg);
                }
                else if (bag_message->topic_name == keyframeTopic) {
                    auto header_msg = std::make_shared<std_msgs::msg::Header>();
                    header_ser.deserialize_message(&serialized_msg, header_msg.get());
                    
                    MO->keyframeHandler(header_msg);
                }
                else {
                    auto it = std::find(pointCloudTopics.begin(), pointCloudTopics.end(), bag_message->topic_name);
                    if (it == pointCloudTopics.end()) continue;
                    int idx = std::distance(pointCloudTopics.begin(), it);

                    if (current_bag_time - (first_msg_time / 1e9) < skipLidarTopic)
                        continue;

                    auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                    pc_ser.deserialize_message(&serialized_msg, pc_msg.get());
                    
                    IPs[idx]->cloudHandler(pc_msg);

                    if (lidarSync && loopClosureEnableFlag && loopClosureFrequency > 0.0) {
                        double loop_period = 1.0 / loopClosureFrequency;
                        
                        if (loopClosureThread) {
                            if (current_bag_time - last_loop_closure_time >= loop_period) {
                                last_loop_closure_time = current_bag_time;
                                
                                if (!is_loop_closing.load()) {
                                    is_loop_closing.store(true);
                                    
                                    std::thread([this]() {
                                        if(useScanContext) {
                                            MO->performRSLoopClosure();
                                            MO->performSCLoopClosure();
                                        }
                                        else {
                                            MO->performLoopClosure();
                                        }

                                        MO->visualizeLoopClosure();
                                        is_loop_closing.store(false);
                                    }).detach();
                                }
                            }
                        }
                        else {
                            if(useScanContext) {
                                MO->performRSLoopClosure();
                                MO->performSCLoopClosure();
                            }
                            else {
                                MO->performLoopClosure();
                            }
                            MO->visualizeLoopClosure();
                        }
                    }
                    lidarSync = false;
                }

                std::this_thread::yield();
            }
        });

        while (reader.has_next() && rclcpp::ok()) {
            bag_queue_.push(reader.read_next());
        }

        bag_queue_.push(nullptr);

        compute_thread.join();

        if (loopClosureThread && is_loop_closing.load()) {
            RCLCPP_INFO(this->get_logger(), "等待最後一次回環檢測運算結束...");
            while (is_loop_closing.load() && rclcpp::ok()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        if (saveFullKeyFrame) {
            string frameDirectory = std::getenv("HOME") + saveFrameDirectory;
            pcl::io::savePCDFileBinary(frameDirectory + "trajectory.pcd", *MO->cloudKeyPoses6D);
            RCLCPP_INFO(this->get_logger(), "軌跡已匯出至: %strajectory.pcd", frameDirectory.c_str());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);

    if (args.size() < 2) {
        std::cerr << "Usage: ros2 run lio_sam lio_sam_offline_runner <bag_path> --ros-args --params-file <path_to_params.yaml>" << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    std::string bag_path = args[1];
    rclcpp::NodeOptions options;
    
    // 這裡會阻塞，直到整個 Bag 全速跑完
    auto node = std::make_shared<OfflineRunner>(options, bag_path);

    node->start_http_server(); 
    node->run(bag_path);

    // ==========================================
    // 【駐留監聽】跑完後印出提示，並啟動 Executor 監聽 Service
    // ==========================================
    RCLCPP_INFO(node->get_logger(), "=========================================================");
    RCLCPP_INFO(node->get_logger(), "🎉 Bag 處理完畢！地圖與 TF 已保留在記憶體中。");
    RCLCPP_INFO(node->get_logger(), "您可以現在打開 RViz 觀看最終點雲與軌跡。");
    RCLCPP_INFO(node->get_logger(), "若要存檔，請在『另一個終端機』執行以下指令：");
    RCLCPP_INFO(node->get_logger(), "ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap \"{resolution: 0.0, destination: '/Downloads/LOAM/'}\"");
    RCLCPP_INFO(node->get_logger(), "存檔完成後，在此視窗按 Ctrl+C 即可結束程式。");
    RCLCPP_INFO(node->get_logger(), "=========================================================");

    // 使用 MultiThreadedExecutor 將所有 Node (包含持有 service 的 mapOptimization) 包起來 Spin
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(node->MO);
    
    // 開始掛機，等待呼叫
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
