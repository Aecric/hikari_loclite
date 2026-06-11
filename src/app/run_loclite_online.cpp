#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

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

    // 与 lightning 定位模式一致的 CLI:
    //   ros2 run hikari_loclite run_loclite_online --config <yaml> --map_path <map_dir>
    // --config 为空时退回 ROS param config_path (launch 文件兼容);
    // --map_path 非空时覆盖 yaml 中的地图目录 (从 <map_path>/global.pcd 加载固定地图)
    const std::string config = ParseFlag(argc, argv, "--config");
    const std::string map_path = ParseFlag(argc, argv, "--map_path");

    auto node = std::make_shared<hikari::loclite::LocLiteNode>();

    if (!node->Init(config, map_path)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to init hikari_loclite");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "hikari_loclite running");
    rclcpp::spin(node);
    node->Shutdown();
    rclcpp::shutdown();
    return 0;
}
