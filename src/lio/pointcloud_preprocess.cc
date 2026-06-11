#include "lio/pointcloud_preprocess.h"
#include <cmath>
#include <execution>
#include "log.h"

namespace hikari::loclite {

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

void PointCloudPreprocess::Process(const sensor_msgs::msg::PointCloud2::SharedPtr& msg, PointCloudType::Ptr& pcl_out) {
    cloud_out_.clear();
    cloud_full_.clear();

    switch (lidar_type_) {
        case LidarType::OUST64:
            Oust64Handler(msg);
            break;
        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;
        case LidarType::ROBOSENSE:
            RoboSenseHandler(msg);
            break;
        default:
            LOG(ERROR) << "PointCloud2 input is unsupported for Livox/AVIA lidar_type; use livox CustomMsg";
            break;
    }
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Process(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, PointCloudType::Ptr& pcl_out) {
    cloud_out_.clear();
    cloud_full_.clear();

    int plsize = msg->point_num;
    if (plsize <= 1) {
        *pcl_out = cloud_out_;
        return;
    }
    cloud_out_.reserve(plsize);
    cloud_full_.resize(plsize);

    std::vector<char> is_valid_pt(plsize, 0);
    std::vector<uint> index(plsize - 1);
    for (uint i = 0; i < plsize - 1; ++i) index[i] = i + 1;

    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint& i) {
        if (i % point_filter_num_ != 0) return;

        cloud_full_[i].x = msg->points[i].x;
        cloud_full_[i].y = msg->points[i].y;
        cloud_full_[i].z = msg->points[i].z;
        cloud_full_[i].intensity = msg->points[i].reflectivity;
        cloud_full_[i].time = msg->points[i].offset_time / double(1000000);

        if (cloud_full_[i].z < height_min_ || cloud_full_[i].z > height_max_) return;

        const float x = cloud_full_[i].x;
        const float y = cloud_full_[i].y;
        const float z = cloud_full_[i].z;
        const double r2 = double(x) * x + double(y) * y + double(z) * z;
        if (!std::isfinite(r2) || r2 < blind_ * blind_ || r2 > max_range_ * max_range_) return;

        const bool dedup = (std::abs(x - cloud_full_[i - 1].x) > 1e-7f) ||
                           (std::abs(y - cloud_full_[i - 1].y) > 1e-7f) ||
                           (std::abs(z - cloud_full_[i - 1].z) > 1e-7f);
        if (dedup) is_valid_pt[i] = 1;
    });

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) cloud_out_.points.push_back(cloud_full_[i]);
    }

    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
    *pcl_out = cloud_out_;
}

void PointCloudPreprocess::Oust64Handler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    cloud_out_.reserve(pl_orig.size());

    for (int i = 0; i < (int)pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) continue;
        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        if (!std::isfinite(range) || range < (blind_ * blind_) || range > (max_range_ * max_range_)) continue;
        if (pl_orig.points[i].z < height_min_ || pl_orig.points[i].z > height_max_) continue;

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = pl_orig.points[i].t / 1e6;
        cloud_out_.points.push_back(added_pt);
    }
    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::RoboSenseHandler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<PointRobotSense> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    cloud_out_.reserve(pl_orig.size());

    double head_time = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;

    for (int i = 0; i < (int)pl_orig.points.size(); i++) {
        if (i % point_filter_num_ != 0) continue;
        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        if (!std::isfinite(range) || range < (blind_ * blind_) || range > (max_range_ * max_range_)) continue;
        if (pl_orig.points[i].z < height_min_ || pl_orig.points[i].z > height_max_) continue;

        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = (pl_orig.points[i].timestamp - head_time) * 1e3;
        cloud_out_.points.push_back(added_pt);
    }
    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

void PointCloudPreprocess::VelodyneHandler(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    double omega_l = 3.61;
    std::vector<bool> is_first(num_scans_, true);
    std::vector<double> yaw_fp(num_scans_, 0.0);
    std::vector<float> yaw_last(num_scans_, 0.0);
    std::vector<float> time_last(num_scans_, 0.0);

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time_ = true;
    } else {
        given_offset_time_ = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.time = pl_orig.points[i].time * time_scale_;

        if (!given_offset_time_) {
            int layer = pl_orig.points[i].ring;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.time = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.time;
                continue;
            }
            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.time = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.time = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }
            if (added_pt.time < time_last[layer]) added_pt.time += 360.0 / omega_l;
            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.time;
        }

        if (i % point_filter_num_ == 0) {
            const double r2 = double(added_pt.x) * added_pt.x + double(added_pt.y) * added_pt.y +
                              double(added_pt.z) * added_pt.z;
            if (std::isfinite(r2) && r2 > (blind_ * blind_) && r2 < (max_range_ * max_range_)) {
                cloud_out_.points.push_back(added_pt);
            }
        }
    }
    cloud_out_.width = cloud_out_.size();
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
}

}  // namespace hikari::loclite
