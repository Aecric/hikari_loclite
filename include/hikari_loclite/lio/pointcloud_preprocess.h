#pragma once

#ifndef HIKARI_LOCLITE_POINTCLOUD_PREPROCESS_H
#define HIKARI_LOCLITE_POINTCLOUD_PREPROCESS_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/measure_group.h"
#include "common/point_def.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace hikari::loclite {

enum class LidarType { AVIA = 1, VELO32 = 2, OUST64 = 3, ROBOSENSE = 4 };

class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    void Process(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, PointCloudType::Ptr& pcl_out);
    void Process(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud, PointCloudType::Ptr& pcl_out);

    void Set(LidarType lid_type, double bld, int pfilt_num);

    double& Blind() { return blind_; }
    double& MaxRange() { return max_range_; }
    int& NumScans() { return num_scans_; }
    int& PointFilterNum() { return point_filter_num_; }
    float& TimeScale() { return time_scale_; }
    LidarType GetLidarType() const { return lidar_type_; }
    void SetLidarType(LidarType lt) { lidar_type_ = lt; }

    void SetHeightROI(float height_max, float height_min) {
        height_max_ = height_max;
        height_min_ = height_min;
    }

   private:
    void Oust64Handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void RoboSenseHandler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    void VelodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);

    PointCloudType cloud_full_, cloud_out_;

    LidarType lidar_type_ = LidarType::AVIA;
    int point_filter_num_ = 1;
    int num_scans_ = 6;
    double blind_ = 0.01;
    double max_range_ = 60.0;
    float time_scale_ = 1e-3;
    bool given_offset_time_ = false;
    float height_max_ = 1.0;
    float height_min_ = -1.0;
};

}  // namespace hikari::loclite

#endif
