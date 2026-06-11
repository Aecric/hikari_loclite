//
// run_loclite_offline — 离线定位评估节点
//
// 用 rosbag2_cpp SequentialReader 顺序读 bag, 按 bag 顺序把 IMU / Livox / PointCloud2
// 喂给与 online 完全相同的 LocLiteNode 处理路径, 正常发布全部话题 / TF (便于录制评估),
// 读完打印帧数与最终状态后退出.
//
//   ros2 run hikari_loclite run_loclite_offline --config <yaml> --input_bag <bag> [--map_path <dir>]
//
// 注意: 契约文档 hikari_loclite_build_2026-06-10.md 要求 "在 release 模式下 offline 节点不编译",
//       本目标仅在非 Release 构建时由 CMakeLists.txt 编译/安装.
//

#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <yaml-cpp/yaml.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "system/loclite_node.hpp"

namespace {

/// 手写 argv 解析 (gflags 风格但不引入 gflags): 同时支持 --flag value 与 --flag=value
std::string ParseFlag(int argc, char** argv, const std::string& flag) {
    const std::string with_eq = flag + "=";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == flag && i + 1 < argc) {
            return argv[i + 1];
        }
        if (arg.rfind(with_eq, 0) == 0) {
            return arg.substr(with_eq.size());
        }
    }
    return "";
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    const std::string config = ParseFlag(argc, argv, "--config");
    const std::string input_bag = ParseFlag(argc, argv, "--input_bag");
    const std::string map_path = ParseFlag(argc, argv, "--map_path");

    if (config.empty() || input_bag.empty()) {
        std::cerr << "Usage: run_loclite_offline --config <yaml> --input_bag <bag> [--map_path <dir>]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    auto node = std::make_shared<hikari::loclite::LocLiteNode>();
    if (!node->Init(config, map_path)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to init hikari_loclite (offline)");
        rclcpp::shutdown();
        return 1;
    }

    // 话题名取自 yaml common 段 (与 online 订阅同源)
    std::string imu_topic = "/livox/imu";
    std::string livox_topic = "/livox/lidar";
    std::string cloud_topic = "/cloud";
    try {
        auto yaml = YAML::LoadFile(config);
        if (yaml["common"]) {
            imu_topic = yaml["common"]["imu_topic"].as<std::string>(imu_topic);
            livox_topic = yaml["common"]["livox_lidar_topic"].as<std::string>(livox_topic);
            cloud_topic = yaml["common"]["pointcloud_topic"].as<std::string>(cloud_topic);
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node->get_logger(), "Failed to read topics from %s: %s (using defaults)", config.c_str(),
                    e.what());
    }

    rosbag2_cpp::Reader reader;
    try {
        reader.open(input_bag);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open bag %s: %s", input_bag.c_str(), e.what());
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "replaying bag %s (imu=%s, livox=%s, cloud=%s)", input_bag.c_str(),
                imu_topic.c_str(), livox_topic.c_str(), cloud_topic.c_str());

    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serde;
    rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> livox_serde;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serde;

    size_t imu_count = 0, livox_count = 0, cloud_count = 0;
    while (rclcpp::ok() && reader.has_next()) {
        auto bag_msg = reader.read_next();
        if (bag_msg->topic_name == imu_topic) {
            rclcpp::SerializedMessage smsg(*bag_msg->serialized_data);
            auto msg = std::make_shared<sensor_msgs::msg::Imu>();
            imu_serde.deserialize_message(&smsg, msg.get());
            node->FeedImu(msg);
            ++imu_count;
        } else if (bag_msg->topic_name == livox_topic) {
            rclcpp::SerializedMessage smsg(*bag_msg->serialized_data);
            auto msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
            livox_serde.deserialize_message(&smsg, msg.get());
            node->FeedLivox(msg);
            ++livox_count;
        } else if (bag_msg->topic_name == cloud_topic) {
            rclcpp::SerializedMessage smsg(*bag_msg->serialized_data);
            auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            cloud_serde.deserialize_message(&smsg, msg.get());
            node->FeedCloud(msg);
            ++cloud_count;
        } else {
            continue;
        }

        // 周期性 spin: 处理回放期间的 /initialpose 注入与 sc_reloc 服务调用
        if ((imu_count + livox_count + cloud_count) % 50 == 0) {
            rclcpp::spin_some(node);
        }
    }

    RCLCPP_INFO(node->get_logger(), "bag done: imu=%zu, livox=%zu, cloud=%zu, final state=%s(%d)", imu_count,
                livox_count, cloud_count, node->CurrentStateStr(), static_cast<int>(node->CurrentState()));

    node->Shutdown();
    rclcpp::shutdown();
    return 0;
}
