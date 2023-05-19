//
// Created by qzj on 23-2-22.
//
#pragma once
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kitti_utils {
    
    std::string GetPCDPath(const std::string& kitti_root, const int &seq, const int &frame_id);

    std::string GetLabelPath(const std::string& kitti_root, const int &seq, const int &frame_id, const std::string& label_dir = "labels");

    pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud(const std::string& kitti_root, const int &seq, const int &frame_id);

    pcl::PointCloud<pcl::PointXYZI>::Ptr GetCloudI(const std::string& kitti_root, const int &seq, const int &frame_id);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ReadSemCloud(const std::string& kitti_root, const int &seq, const int &frame_id,
                                                      const std::string& label_dir = "labels");

    void SaveSemLabel(const std::string& kitti_root, const int &seq, const int &frame_id,
                      const pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud, const std::string& label_dir = "labels");

    Eigen::Matrix4d GetTr(const std::string& kitti_root, int seq);

    Eigen::Matrix4d getLidarPose(const std::string& kitti_root, const int &seq, const int &frame_id);

    bool isGround(const uint16_t sem_id);

    bool isPillar(const uint16_t sem_id);

    bool isPlane(const uint16_t sem_id);
}