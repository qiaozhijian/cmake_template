//
// Created by qzj on 2021/4/25.
//

#ifndef SRC_CONFIG_H
#define SRC_CONFIG_H

#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string>
#include <glog/logging.h>
#include <tic_toc.h>
#include <file_manager.h>

namespace config {
    // benchmark configs
    extern std::string project_path;
    extern std::string config_file;

    extern double max_range;
    extern double min_range;
    extern std::string kitti_root;
    extern std::string label_dir;
    extern std::string split_dir;

    void readParameters(std::string config_file);

}

void InitGLOG(std::string argv0);

#endif //SRC_CONFIG_H
