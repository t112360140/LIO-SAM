#include <rclcpp/rclcpp.hpp>

#include "lio_sam/utility.hpp"
#include "lib/httplib.h"
#include "lib/json.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>

#include <list>
#include <unordered_map>
#include <mutex>
#include <algorithm>

#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>


#pragma pack(push, 1)
struct WebGLPoint {
    float x, y, z;
    uint8_t r, g, b, a;
};
#pragma pack(pop)

class MapChunkManager {
private:
    std::string base_dir_;
    bool is_chunked_;
    float min_intensity_;
    float max_intensity_;
    size_t max_cache_size_;

    std::list<std::string> lru_list_;
    
    struct CacheEntry {
        pcl::PointCloud<PointType>::Ptr cloud;
        bool has_color; 
        std::list<std::string>::iterator list_it;
    };
    std::unordered_map<std::string, CacheEntry> cache_;
    std::mutex cache_mutex_;

    void getPseudoColor(float value, float vmin, float vmax, uint8_t& r, uint8_t& g, uint8_t& b) {
        float c = std::max(0.0f, std::min(1.0f, (value - vmin) / (vmax - vmin)));
        r = static_cast<uint8_t>(std::max(0.0f, std::min(1.0f, 1.5f - std::abs(4.0f * c - 3.0f))) * 255.0f);
        g = static_cast<uint8_t>(std::max(0.0f, std::min(1.0f, 1.5f - std::abs(4.0f * c - 2.0f))) * 255.0f);
        b = static_cast<uint8_t>(std::max(0.0f, std::min(1.0f, 1.5f - std::abs(4.0f * c - 1.0f))) * 255.0f);
    }

    bool loadChunkFromDisk(const std::string& chunk_id, pcl::PointCloud<PointType>::Ptr& out_cloud, bool force_intensity, bool& has_color) {
        std::string file_path;
        std::string color_file_path;
        
        if (!is_chunked_ || chunk_id == "all") {
            file_path = base_dir_ + "GlobalMap.pcd";
            color_file_path = base_dir_ + "GlobalMap_Color.pcd";
        } else {
            file_path = base_dir_ + "GlobalMap_Chunks/" + chunk_id + ".pcd";
            color_file_path = base_dir_ + "GlobalMap_Color_Chunks/" + chunk_id + ".pcd";
        }

        out_cloud.reset(new pcl::PointCloud<PointType>());
        has_color = false;

        bool color_exists = std::filesystem::exists(color_file_path);
        bool mono_exists = std::filesystem::exists(file_path);

        if (!force_intensity && color_exists) {
            if (pcl::io::loadPCDFile<PointType>(color_file_path, *out_cloud) != -1) {
                has_color = true;
                return true;
            }
        }
        
        if (mono_exists) {
            if (pcl::io::loadPCDFile<PointType>(file_path, *out_cloud) != -1) {
                return true;
            }
        }
        
        return false;
    }

public:
    MapChunkManager(const std::string& base_dir, bool is_chunked, float min_int, float max_int, size_t max_cache_size = 100) 
        : base_dir_(base_dir), is_chunked_(is_chunked), max_cache_size_(max_cache_size), min_intensity_(min_int), max_intensity_(max_int) {}

    std::vector<WebGLPoint> getChunkData(const std::string& chunk_id, int stride, bool force_intensity = false) {
        pcl::PointCloud<PointType>::Ptr target_cloud = nullptr;
        bool has_color = false;

        std::string cache_key = chunk_id + (force_intensity ? "_mono" : "_color");

        {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            auto it = cache_.find(cache_key);
            if (it != cache_.end()) {
                // Cache Hit
                lru_list_.erase(it->second.list_it);
                lru_list_.push_front(cache_key);
                it->second.list_it = lru_list_.begin();
                target_cloud = it->second.cloud;
                has_color = it->second.has_color;
            } else {
                // Cache Miss
                if (loadChunkFromDisk(chunk_id, target_cloud, force_intensity, has_color)) {
                    if (cache_.size() >= max_cache_size_) {
                        std::string oldest = lru_list_.back();
                        lru_list_.pop_back();
                        cache_.erase(oldest);
                    }
                    lru_list_.push_front(cache_key);
                    cache_[cache_key] = {target_cloud, has_color, lru_list_.begin()};
                }
            }
        }

        std::vector<WebGLPoint> buffer;
        if (!target_cloud) return buffer;

        stride = std::max(1, stride);
        buffer.reserve(target_cloud->size() / stride);

        for (size_t i = 0; i < target_cloud->size(); i += stride) {
            const auto& pt = target_cloud->points[i];
            WebGLPoint wp;
            wp.x = pt.x;
            wp.y = pt.y;
            wp.z = pt.z;
            wp.a = 255;

            if (has_color) {
                wp.r = pt.r;
                wp.g = pt.g;
                wp.b = pt.b;
            } else {
                getPseudoColor(pt.intensity, min_intensity_, max_intensity_, wp.r, wp.g, wp.b);
            }
            buffer.push_back(wp);
        }
        return buffer;
    }
};

class MapViewerServerNode : public ParamServer {
public:
    MapViewerServerNode(const rclcpp::NodeOptions & options) 
        : ParamServer("map_viewer_server", options) 
    {
        std::string base_path = std::getenv("HOME") + savePCDDirectory;
        bool is_chunked = (chuckSize > 0.0);

        chunk_manager_ = std::make_shared<MapChunkManager>(base_path, is_chunked, minIntensity, maxIntensity, 100);

        start_server();
    }

private:
    std::shared_ptr<MapChunkManager> chunk_manager_;
    std::thread server_thread_;
    httplib::Server svr_;
    bool splitZ_;

    void start_server() {
        std::string pkg_share_dir = ament_index_cpp::get_package_share_directory(PROJECT_NAME);
        std::string web_dir = pkg_share_dir + "/web" + "/viewer";
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

        svr_.set_mount_point("/",  web_dir);

        svr_.Get("/config", [this](const httplib::Request &req, httplib::Response &res) {
            res.set_header("Access-Control-Allow-Origin", "*");
            
            nlohmann::json config_json = {
                {"isChunked", (chuckSize > 0.0)},
                {"chunkSize", chuckSize},
                {"splitZ", splitZ_},
                {"minIntensity", minIntensity},
                {"maxIntensity", maxIntensity}
            };
            res.set_content(config_json.dump(), "application/json");
        });

        svr_.Get(R"(/chunk/(.*))", [this](const httplib::Request &req, httplib::Response &res) {
            res.set_header("Access-Control-Allow-Origin", "*");

            std::string chunk_id = req.matches[1]; // 例: "0_1_2" 或 "all"
            
            int stride = 1; 
            if (req.has_param("stride")) stride = std::stoi(req.get_param_value("stride"));
            
            bool force_intensity = false;
            if (req.has_param("color")) force_intensity = (req.get_param_value("color") == "false");

            auto data = chunk_manager_->getChunkData(chunk_id, stride, force_intensity);

            if (data.empty()) {
                res.set_content("", 0, "application/octet-stream");
                return;
            }

            res.set_content(reinterpret_cast<const char*>(data.data()), 
                            data.size() * sizeof(WebGLPoint), 
                            "application/octet-stream");
        });

        server_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "🌐 Map Viewer 伺服器啟動: http://0.0.0.0:8081");
            svr_.listen("0.0.0.0", 8081);
        });
        server_thread_.detach();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<MapViewerServerNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}