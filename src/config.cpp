//
// Created by qzj on 2021/12/24.
//

#include "config.h"
#include <glog/logging.h>
#include "global_definition/global_definition.h"

namespace config {

    // benchmark configs
    std::string project_path = config::WORK_SPACE_PATH;
    std::string config_file, kitti_root, label_dir, split_dir;
    double max_range, min_range;

    template<typename T>
    T get(const YAML::Node &node, const std::string &key, const T &default_value) {
        if (!node[key]) {
            LOG(INFO) << "Key " << key << " not found, using default value: " << default_value;
            return default_value;
        }
        T value = node[key].as<T>();
        LOG(INFO) << "Key " << key << " found, using value: " << value;
        return value;
    }

    template<typename T>
    T get(const YAML::Node &node, const std::string &father_key, const std::string &key, const T &default_value) {
        if (!node[father_key] || !node[father_key][key]) {
            LOG(INFO) << "Key " << father_key << "/" << key << " not found, using default value: " << default_value;
            return default_value;
        }
        T value = node[father_key][key].as<T>();
        LOG(INFO) << "Key " << father_key << "/" << key << " found, using value: " << value;
        return value;
    }

    void readParameters(std::string config_file_) {
        config_file = config_file_;
        LOG(INFO) << "config_path: " << config_file;
        std::ifstream fin(config_file);
        if (!fin) {
            std::cout << "config_file: " << config_file << " not found." << std::endl;
            return;
        }
        YAML::Node config_node = YAML::LoadFile(config_file);

        kitti_root = get(config_node, "dataset", "dataset_path", std::string(""));
        split_dir = get(config_node, "dataset", "split_dir", std::string(""));
        label_dir = get(config_node, "dataset", "label_dir", std::string(""));
        max_range = get(config_node, "dataset", "max_range", 120.0);
        min_range = get(config_node, "dataset", "min_range", 0.5);
    }
}

void InitGLOG(std::string file_name){
    FileManager::CreateDirectory(config::WORK_SPACE_PATH + "/Log");
    FileManager::CreateDirectory(config::WORK_SPACE_PATH + "/Log/logs");
    google::InitGoogleLogging("glog");
    google::SetLogFilenameExtension((std::string("_") +file_name + std::string(".txt")).c_str());
    FLAGS_log_dir = config::WORK_SPACE_PATH + "/Log/logs";
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = false;
    FLAGS_log_prefix = false;
    FLAGS_logbufsecs = 0;
}