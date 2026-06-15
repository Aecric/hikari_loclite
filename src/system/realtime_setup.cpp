// sched_setaffinity / cpu_set_t 需要 _GNU_SOURCE, 必须在任何系统头之前定义.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "system/realtime_setup.hpp"

#include <pthread.h>
#include <sched.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace hikari::loclite {

void ApplyRealtimeScheduling(const RealtimeOptions& opts, const rclcpp::Logger& logger) {
    if (!opts.enabled) {
        RCLCPP_INFO(logger, "realtime scheduling disabled (system.rt_enabled=false)");
        return;
    }

    // --- CPU 亲和 ---
    if (!opts.cpu_cores.empty()) {
        cpu_set_t set;
        CPU_ZERO(&set);
        const long ncpu = sysconf(_SC_NPROCESSORS_ONLN);
        std::string cores_str;
        for (int core : opts.cpu_cores) {
            if (core < 0 || (ncpu > 0 && core >= ncpu)) {
                RCLCPP_WARN(logger, "rt_cpu_cores: core %d out of range [0,%ld), skipped", core, ncpu);
                continue;
            }
            CPU_SET(core, &set);
            cores_str += std::to_string(core) + " ";
        }
        if (CPU_COUNT(&set) > 0) {
            if (sched_setaffinity(0, sizeof(set), &set) == 0) {
                RCLCPP_INFO(logger, "CPU affinity set to cores: %s", cores_str.c_str());
            } else {
                RCLCPP_WARN(logger, "sched_setaffinity failed (%s); CPU affinity not applied",
                            std::strerror(errno));
            }
        }
    }

    // --- 实时调度优先级 ---
    if (opts.sched_fifo) {
        sched_param sp{};
        int prio = opts.priority;
        const int pmin = sched_get_priority_min(SCHED_FIFO);
        const int pmax = sched_get_priority_max(SCHED_FIFO);
        if (pmin >= 0 && pmax >= 0) {
            prio = std::max(pmin, std::min(prio, pmax));
        }
        sp.sched_priority = prio;
        const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        if (rc == 0) {
            RCLCPP_INFO(logger, "SCHED_FIFO priority set to %d", prio);
        } else {
            // 最常见原因是缺 cap_sys_nice (开发机直接 ros2 run 时正常出现) —— 仅告警, 继续按普通调度跑.
            RCLCPP_WARN(logger,
                        "pthread_setschedparam(SCHED_FIFO, %d) failed (%s); running with default "
                        "scheduling. Needs cap_sys_nice: sudo setcap cap_sys_nice+ep <binary>",
                        prio, std::strerror(rc));
        }
    }
}

}  // namespace hikari::loclite
