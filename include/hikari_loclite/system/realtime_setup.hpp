#pragma once

#ifndef HIKARI_LOCLITE_REALTIME_SETUP_HPP
#define HIKARI_LOCLITE_REALTIME_SETUP_HPP

#include <vector>

#include <rclcpp/logger.hpp>

namespace hikari::loclite {

/// CPU 亲和 + 实时调度参数 (来自 yaml system.rt_*). 默认全关, 产线模板再开.
struct RealtimeOptions {
    bool enabled = false;          // 总开关; false 时 ApplyRealtimeScheduling 直接返回
    std::vector<int> cpu_cores;    // 绑定的 CPU 核列表; 空 = 不绑核
    bool sched_fifo = false;       // 是否切到 SCHED_FIFO 实时调度策略
    int priority = 80;             // SCHED_FIFO 优先级 (会被 clamp 到内核允许区间)
};

/// 对调用线程 (即 main/spin 线程) 设置 CPU 亲和与实时调度.
/// 需要 cap_sys_nice (由 .deb postinst setcap 授予, 见 docker2/postinst.in);
/// 权限不足或部分核越界时打印 warning 并继续, **不视为失败** —— 开发机/bag 回放零门槛可跑.
void ApplyRealtimeScheduling(const RealtimeOptions& opts, const rclcpp::Logger& logger);

}  // namespace hikari::loclite

#endif  // HIKARI_LOCLITE_REALTIME_SETUP_HPP
