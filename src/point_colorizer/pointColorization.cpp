#include "lio_sam/utility.hpp"
#include "lio_sam/msg/cloud_info.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <image_geometry/pinhole_camera_model.h>

#ifdef HAS_CUDA
#include "point_colorizer.cuh"
#include <cuda_runtime.h>
#endif

struct ImageFrame {
    double time_sec;
    cv::Mat image;
    std_msgs::msg::Header header;
};

class PointColorization : public ParamServer
{
private:
    std::mutex mtx;
    std::vector<std::deque<ImageFrame>> image_caches_;
    std::vector<image_geometry::PinholeCameraModel> camera_models_;
    std::vector<bool> maps_initialized_;
    
    // CPU 去畸變 Map
    std::vector<cv::Mat> map_x_;
    std::vector<cv::Mat> map_y_;

    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> rect_image_pubs_;
    std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> fov_marker_pubs_;
    
    const int MAX_POINTS = 300000*10;
    
#ifdef HAS_CUDA
    float *um_x_, *um_y_, *um_z_;
    uint8_t *um_r_, *um_g_, *um_b_;
    bool *um_is_colored_, *um_is_removed_;
    uint8_t *um_raw_image_, *um_rect_image_;
    
    // GPU 去畸變 Map
    std::vector<float*> um_map_x_;
    std::vector<float*> um_map_y_;
    const int MAX_IMAGE_SIZE = 1920 * 1080 * 4;
#endif

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::function<void(const lio_sam::msg::CloudInfo::SharedPtr)> direct_cloud_info_callback;

    void setCloudInfoCallback(std::function<void(const lio_sam::msg::CloudInfo::SharedPtr)> cb) {
        direct_cloud_info_callback = cb;
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    PointColorization(const rclcpp::NodeOptions & options) : ParamServer("lio_sam_pointColorization", options)
    {
        int num_cams = imageTopics.size();
        image_caches_.resize(num_cams);
        camera_models_.resize(num_cams);
        maps_initialized_.resize(num_cams, false);
        
        map_x_.resize(num_cams);
        map_y_.resize(num_cams);

        for (int i = 0; i < num_cams; ++i) {
            rect_image_pubs_.push_back(
                this->create_publisher<sensor_msgs::msg::Image>("lio_sam/image/camera_" + std::to_string(i), 1)
            );
            fov_marker_pubs_.push_back(
                this->create_publisher<visualization_msgs::msg::Marker>("lio_sam/marker/camera_fov_" + std::to_string(i), 1)
            );
        }
        
#ifdef HAS_CUDA
        um_map_x_.resize(num_cams, nullptr);
        um_map_y_.resize(num_cams, nullptr);

        if (useCudaColorization) {
            cudaMallocManaged(&um_x_, MAX_POINTS * sizeof(float));
            cudaMallocManaged(&um_y_, MAX_POINTS * sizeof(float));
            cudaMallocManaged(&um_z_, MAX_POINTS * sizeof(float));
            cudaMallocManaged(&um_r_, MAX_POINTS * sizeof(uint8_t));
            cudaMallocManaged(&um_g_, MAX_POINTS * sizeof(uint8_t));
            cudaMallocManaged(&um_b_, MAX_POINTS * sizeof(uint8_t));
            cudaMallocManaged(&um_is_colored_, MAX_POINTS * sizeof(bool));
            cudaMallocManaged(&um_is_removed_, MAX_POINTS * sizeof(bool));
            cudaMallocManaged(&um_raw_image_, MAX_IMAGE_SIZE * sizeof(uint8_t));
            cudaMallocManaged(&um_rect_image_, MAX_IMAGE_SIZE * sizeof(uint8_t));
            RCLCPP_INFO(get_logger(), "\033[1;32m----> Point Colorization Module Initialized (GPU Enabled).\033[0m");
        } else {
            RCLCPP_INFO(get_logger(), "\033[1;32m----> Point Colorization Module Initialized (CPU Mode).\033[0m");
        }
#else
        RCLCPP_INFO(get_logger(), "\033[1;32m----> Point Colorization Module Initialized (CPU Mode).\033[0m");
#endif
    }
    
    ~PointColorization() {
#ifdef HAS_CUDA
        if (useCudaColorization) {
            cudaFree(um_x_); cudaFree(um_y_); cudaFree(um_z_);
            cudaFree(um_r_); cudaFree(um_g_); cudaFree(um_b_);
            cudaFree(um_is_colored_); cudaFree(um_is_removed_);
            cudaFree(um_raw_image_); cudaFree(um_rect_image_);
            for (size_t i = 0; i < um_map_x_.size(); ++i) {
                if (um_map_x_[i]) cudaFree(um_map_x_[i]);
                if (um_map_y_[i]) cudaFree(um_map_y_[i]);
            }
        }
#endif
    }

    void imageHandler(const sensor_msgs::msg::Image::SharedPtr imgMsg, int cam_idx)
    {
        std::lock_guard<std::mutex> lock(mtx);
        try {
            int channels = (imgMsg->encoding == "bgra8" || imgMsg->encoding == "rgba8") ? 4 : 3;
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*imgMsg, (channels == 4) ? "bgra8" : "bgr8");
            
            ImageFrame frame;
            frame.time_sec = imgMsg->header.stamp.sec + imgMsg->header.stamp.nanosec * 1e-9;
            frame.image = cv_ptr->image.clone();
            frame.header = imgMsg->header;
            
            image_caches_[cam_idx].push_back(frame);
            if (image_caches_[cam_idx].size() > 20) image_caches_[cam_idx].pop_front();
        } catch (cv_bridge::Exception& e) { }
    }

    void compressedImageHandler(const sensor_msgs::msg::CompressedImage::SharedPtr imgMsg, int cam_idx)
    {
        std::lock_guard<std::mutex> lock(mtx);
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*imgMsg, "bgr8");
            ImageFrame frame;
            frame.time_sec = imgMsg->header.stamp.sec + imgMsg->header.stamp.nanosec * 1e-9;
            frame.image = cv_ptr->image.clone();
            frame.header = imgMsg->header;
            
            image_caches_[cam_idx].push_back(frame);
            if (image_caches_[cam_idx].size() > 20) image_caches_[cam_idx].pop_front();
        } catch (cv_bridge::Exception& e) { }
    }

    void cameraInfoHandler(const sensor_msgs::msg::CameraInfo::SharedPtr infoMsg, int cam_idx)
    {
        std::lock_guard<std::mutex> lock(mtx);
        
        // 只有第一次收到時會去初始化畸變映射表
        if (!maps_initialized_[cam_idx]) {
            camera_models_[cam_idx].fromCameraInfo(infoMsg);
            cv::initUndistortRectifyMap(
                camera_models_[cam_idx].intrinsicMatrix(), camera_models_[cam_idx].distortionCoeffs(),
                camera_models_[cam_idx].rotationMatrix(), camera_models_[cam_idx].projectionMatrix(),
                cv::Size(infoMsg->width, infoMsg->height), CV_32FC1, 
                map_x_[cam_idx], map_y_[cam_idx]);
                
#ifdef HAS_CUDA
            if (useCudaColorization) {
                int map_size = infoMsg->width * infoMsg->height * sizeof(float);
                cudaMallocManaged(&um_map_x_[cam_idx], map_size);
                cudaMallocManaged(&um_map_y_[cam_idx], map_size);
                std::memcpy(um_map_x_[cam_idx], map_x_[cam_idx].data, map_size);
                std::memcpy(um_map_y_[cam_idx], map_y_[cam_idx].data, map_size);
            }
#endif
            maps_initialized_[cam_idx] = true;
            RCLCPP_INFO(this->get_logger(), "相機 %d 的畸變校正映射表 (Map) 建立完成！", cam_idx);
        }
    }

    void cloudInfoHandler(const lio_sam::msg::CloudInfo::SharedPtr msgIn)
    {
        if (!enableColorization) {
            if (direct_cloud_info_callback) direct_cloud_info_callback(msgIn);
            return;
        }

        std::lock_guard<std::mutex> lock(mtx);
        pcl::PointCloud<PointType>::Ptr deskewedCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloud(new pcl::PointCloud<PointType>());

        pcl::fromROSMsg(msgIn->cloud_deskewed, *deskewedCloud);
        pcl::fromROSMsg(msgIn->cloud_corner, *cornerCloud);
        pcl::fromROSMsg(msgIn->cloud_surface, *surfaceCloud);

        double pc_time = msgIn->header.stamp.sec + msgIn->header.stamp.nanosec * 1e-9;

        std::vector<ImageFrame> best_images(imageTopics.size());
        std::vector<bool> image_ready(imageTopics.size(), false);

        for (size_t c = 0; c < imageTopics.size(); ++c) {
            if (!maps_initialized_[c] || image_caches_[c].empty()) continue;

            double min_diff = 1e9;
            for (const auto& frame : image_caches_[c]) {
                double img_corrected_time = frame.time_sec - cameraDelay; 
                double diff = std::abs(pc_time - img_corrected_time);
                
                if (diff < min_diff && diff <= cameraSyncDelayWindow) {
                    min_diff = diff;
                    best_images[c] = frame;
                    image_ready[c] = true;
                }
            }
        }

        std::vector<Eigen::Matrix4f> T_cams(imageTopics.size(), Eigen::Matrix4f::Identity());
        std::vector<bool> tf_ready(imageTopics.size(), false);
        std::string lidar_frame = msgIn->cloud_deskewed.header.frame_id;

        for (size_t c = 0; c < imageTopics.size(); ++c) {
            try {
                geometry_msgs::msg::TransformStamped transform = 
                    tf_buffer_->lookupTransform(cameraFrameIds[c], lidar_frame, tf2::TimePointZero);
                Eigen::Affine3d T_static = tf2::transformToEigen(transform);
                T_cams[c] = T_static.matrix().cast<float>();
                tf_ready[c] = true;
            } catch (const tf2::TransformException & ex) {
                tf_ready[c] = false;
            }
        }

        bool run_cuda = useCudaColorization;
#ifndef HAS_CUDA
        run_cuda = false;
#endif
        if (run_cuda) {
            colorizeWithCUDA(deskewedCloud, best_images, T_cams, tf_ready, image_ready);
            colorizeWithCUDA(cornerCloud, best_images, T_cams, tf_ready, image_ready);
            colorizeWithCUDA(surfaceCloud, best_images, T_cams, tf_ready, image_ready);
        } else {
            colorizeWithCPU(deskewedCloud, cornerCloud, surfaceCloud, best_images, T_cams, tf_ready, image_ready);
        }

        auto original_header = msgIn->cloud_deskewed.header;
        pcl::toROSMsg(*deskewedCloud, msgIn->cloud_deskewed);
        pcl::toROSMsg(*cornerCloud, msgIn->cloud_corner);
        pcl::toROSMsg(*surfaceCloud, msgIn->cloud_surface);
        
        msgIn->cloud_deskewed.header = original_header;
        msgIn->cloud_corner.header = original_header;
        msgIn->cloud_surface.header = original_header;

        if (direct_cloud_info_callback) {
            direct_cloud_info_callback(msgIn);
        }
    }

private:
    void genCamViewMarker(int cam_idx, const image_geometry::PinholeCameraModel& camera_model, const std_msgs::msg::Header& header) {
        if (fov_marker_pubs_[cam_idx]->get_subscription_count() == 0) return;

        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.header.frame_id = cameraFrameIds[cam_idx]; 
        marker.ns = "camera_fov";
        marker.id = cam_idx;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        
        // 設定線條粗細與顏色 (這裡設定為綠色)
        marker.scale.x = 0.05;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8f;

        // 設定視野錐體的深度 (公尺)
        float Z = lidarColorMaxRange; 
        int width = camera_model.cameraInfo().width;
        int height = camera_model.cameraInfo().height;

        // 定義影像的四個角落
        cv::Point2d top_left(0, 0);
        cv::Point2d top_right(width, 0);
        cv::Point2d bottom_right(width, height);
        cv::Point2d bottom_left(0, height);

        // 將 2D 像素座標轉為 3D 射線並縮放至深度 Z
        auto project_to_3d = [&](const cv::Point2d& uv) {
            cv::Point3d ray = camera_model.projectPixelTo3dRay(uv);
            geometry_msgs::msg::Point p;
            p.x = ray.x * (Z / ray.z);
            p.y = ray.y * (Z / ray.z);
            p.z = Z;
            return p;
        };

        geometry_msgs::msg::Point origin; // 相機原點 (0,0,0)

        geometry_msgs::msg::Point tl = project_to_3d(top_left);
        geometry_msgs::msg::Point tr = project_to_3d(top_right);
        geometry_msgs::msg::Point br = project_to_3d(bottom_right);
        geometry_msgs::msg::Point bl = project_to_3d(bottom_left);

        // 構建錐體的四條邊
        marker.points.push_back(origin); marker.points.push_back(tl);
        marker.points.push_back(origin); marker.points.push_back(tr);
        marker.points.push_back(origin); marker.points.push_back(br);
        marker.points.push_back(origin); marker.points.push_back(bl);

        // 構建遠端平面的矩形
        marker.points.push_back(tl); marker.points.push_back(tr);
        marker.points.push_back(tr); marker.points.push_back(br);
        marker.points.push_back(br); marker.points.push_back(bl);
        marker.points.push_back(bl); marker.points.push_back(tl);

        fov_marker_pubs_[cam_idx]->publish(marker);
    }

    void colorizeWithCPU(
        pcl::PointCloud<PointType>::Ptr& deskewed_cloud, pcl::PointCloud<PointType>::Ptr& corner_cloud, pcl::PointCloud<PointType>::Ptr& surface_cloud,
        const std::vector<ImageFrame>& best_images, 
        const std::vector<Eigen::Matrix4f>& T_cams, const std::vector<bool>& tf_ready, const std::vector<bool>& img_ready)
    {        
        // 預先對所有相機的影像去畸變，避免在每個點的迴圈裡重複做
        std::vector<cv::Mat> rect_images(imageTopics.size());
        for (size_t c = 0; c < imageTopics.size(); ++c) {
            if (img_ready[c] && maps_initialized_[c]) {
                cv::remap(best_images[c].image, rect_images[c], map_x_[c], map_y_[c], cv::INTER_LINEAR);
                
                if (rect_image_pubs_[c]->get_subscription_count() > 0) {
                    auto rect_msg = std::make_unique<sensor_msgs::msg::Image>();
                    std::string encoding = (rect_images[c].channels() == 4) ? "bgra8" : "bgr8";
                    cv_bridge::CvImage(best_images[c].header, encoding, rect_images[c]).toImageMsg(*rect_msg);
                    rect_image_pubs_[c]->publish(std::move(rect_msg));
                }
                genCamViewMarker(c, camera_models_[c], best_images[c].header);
            }
        }

        float max_range_sq = lidarColorMaxRange * lidarColorMaxRange;

        auto apply_color_to_cloud = [&](pcl::PointCloud<PointType>::Ptr& cloud) {
            int num_points = cloud->points.size();
            
            #pragma omp parallel for num_threads(numberOfCores)
            for (int i = 0; i < num_points; ++i) {
                auto& pt = cloud->points[i];
                bool colored = false;

                if ((pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > max_range_sq) {
                    pt.r = 0; pt.g = 0; pt.b = 0; pt.a = 0;
                    continue;
                }

                for (size_t c = 0; c < rect_images.size(); ++c) {
                    if (!tf_ready[c] || rect_images[c].empty() || !maps_initialized_[c]) continue;

                    const auto& T = T_cams[c];
                    const auto& model = camera_models_[c]; 
                    int width = rect_images[c].cols;
                    int height = rect_images[c].rows;
                    int step = rect_images[c].step;
                    int channels = rect_images[c].channels();

                    float cam_x = T(0,0)*pt.x + T(0,1)*pt.y + T(0,2)*pt.z + T(0,3);
                    float cam_y = T(1,0)*pt.x + T(1,1)*pt.y + T(1,2)*pt.z + T(1,3);
                    float cam_z = T(2,0)*pt.x + T(2,1)*pt.y + T(2,2)*pt.z + T(2,3);

                    if (cam_z > 0.0f) {
                        int u = std::round((model.fx() * cam_x + model.cx() * cam_z) / cam_z);
                        int v = std::round((model.fy() * cam_y + model.cy() * cam_z) / cam_z);

                        if (u >= 0 && u < width && v >= 0 && v < height) {
                            int img_idx = v * step + u * channels;
                            if (channels == 4 && rect_images[c].data[img_idx + 3] > 128) {
                                continue; 
                            }
                            
                            pt.b = rect_images[c].data[img_idx];
                            pt.g = rect_images[c].data[img_idx + 1];
                            pt.r = rect_images[c].data[img_idx + 2];
                            pt.a = 255;
                            colored = true;
                            break; 
                        }
                    }
                }

                if (!colored) {
                    pt.r = 0; pt.g = 0; pt.b = 0; pt.a = 0; 
                }
            }
        };

        apply_color_to_cloud(deskewed_cloud);
        apply_color_to_cloud(corner_cloud);
        apply_color_to_cloud(surface_cloud);
    }

    void colorizeWithCUDA(pcl::PointCloud<PointType>::Ptr& cloud, const std::vector<ImageFrame>& best_images, 
                          const std::vector<Eigen::Matrix4f>& T_cams, const std::vector<bool>& tf_ready, const std::vector<bool>& img_ready)
    {
#ifdef HAS_CUDA
        int num_points = cloud->points.size();
        if (num_points == 0 || num_points > MAX_POINTS) return;

        for (int i = 0; i < num_points; ++i) {
            um_x_[i] = cloud->points[i].x;
            um_y_[i] = cloud->points[i].y;
            um_z_[i] = cloud->points[i].z;
            um_is_colored_[i] = false;
            um_is_removed_[i] = false;
        }

        for (size_t c = 0; c < best_images.size(); ++c) {
            if (!img_ready[c] || !tf_ready[c] || best_images[c].image.empty() || !maps_initialized_[c]) continue;

            const auto& T = T_cams[c];
            const auto& model = camera_models_[c];
            int width = best_images[c].image.cols;
            int height = best_images[c].image.rows;
            int step = best_images[c].image.step;
            int channels = best_images[c].image.channels();

            int img_size = width * height * channels;
            if (img_size > MAX_IMAGE_SIZE) continue;

            // 將原始影像拷貝至 um_raw_image_
            std::memcpy(um_raw_image_, best_images[c].image.data, img_size);
            
            // 🚀 執行 GPU 去畸變 🚀
            launch_rectify_kernel(
                um_raw_image_, um_rect_image_, um_map_x_[c], um_map_y_[c], 
                width, height, channels);

            if (rect_image_pubs_[c]->get_subscription_count() > 0) {
                cv::Mat rect_image(height, width, (channels == 4) ? CV_8UC4 : CV_8UC3, um_rect_image_);
                auto rect_msg = std::make_unique<sensor_msgs::msg::Image>();
                std::string encoding = (channels == 4) ? "bgra8" : "bgr8";
                cv_bridge::CvImage(best_images[c].header, encoding, rect_image).toImageMsg(*rect_msg);
                rect_image_pubs_[c]->publish(std::move(rect_msg));
            }
            genCamViewMarker(c, camera_models_[c], best_images[c].header);

            float K_array[9] = { (float)model.fx(), 0.0f, (float)model.cx(), 
                                 0.0f, (float)model.fy(), (float)model.cy(), 
                                 0.0f, 0.0f, 1.0f };
            float T_array[12];
            for (int r = 0; r < 3; ++r) {
                for (int col = 0; col < 4; ++col) {
                    T_array[r*4 + col] = T(r, col);
                }
            }

            // 🚀 執行 GPU 上色 (使用去畸變後的 um_rect_image_) 🚀
            launch_colorization_kernel(
                um_x_, um_y_, um_z_, um_r_, um_g_, um_b_, um_is_colored_, um_is_removed_,
                K_array, T_array, um_rect_image_, width, height, step, channels, num_points,
                lidarColorMaxRange
            );
        }

        for (int i = 0; i < num_points; ++i) {
            if (um_is_colored_[i] && !um_is_removed_[i]) {
                cloud->points[i].r = um_r_[i];
                cloud->points[i].g = um_g_[i];
                cloud->points[i].b = um_b_[i];
                cloud->points[i].a = 255;
            } else {
                cloud->points[i].r = 0;
                cloud->points[i].g = 0;
                cloud->points[i].b = 0;
                cloud->points[i].a = 0; 
            }
        }
#endif
    }
};