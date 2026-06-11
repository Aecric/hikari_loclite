//
// Gravity-aligned level frame for obstacle avoidance
// Designed for humanoid robots with gait-induced oscillations
//
// Computes a rotation (R_level_lidar) that transforms lidar points
// into a gravity-aligned "level" frame, with low-pass filtering
// to remove high-frequency gait vibration while preserving terrain slope.
//
// 从 lightning-lm core/lio/gravity_alignment.h 拷贝改造:
//   - namespace lightning → hikari::loclite
//   - 去掉 glog 依赖 (本包日志统一走 RCLCPP, header-only 工具类直接去日志)
//

#pragma once

#ifndef HIKARI_LOCLITE_GRAVITY_ALIGNMENT_H
#define HIKARI_LOCLITE_GRAVITY_ALIGNMENT_H

#include <cmath>
#include <string>

#include "common/eigen_types.h"

namespace hikari::loclite {

/**
 * GravityAlignmentFilter
 *
 * 通过低通滤波重力方向（在雷达坐标系下），计算雷达到水平坐标系的旋转变换。
 *
 * 设计要点：
 *   - 在雷达坐标系中滤波重力矢量，避免角度不连续问题
 *   - 一阶 IIR 低通滤波器，截止频率可配置（默认 0.5Hz）
 *   - 人形机器人步频通常 1~3Hz，0.3~0.5Hz 截止频率可有效分离步态抖动与地形变化
 *   - 只补偿 roll/pitch，不引入 yaw，保证 2D 地图和点云方向一致
 */
class GravityAlignmentFilter {
   public:
    struct Options {
        double cutoff_freq = 0.5;                    // 截止频率 (Hz)
        std::string lidar_frame_id = "lidar_frame";  // 雷达坐标系名称
        std::string level_frame_id = "level_frame";  // 水平坐标系名称
    };

    GravityAlignmentFilter() = default;

    void Init(const Options& options) {
        options_ = options;
        tau_ = 1.0 / (2.0 * M_PI * options_.cutoff_freq);
        initialized_ = false;
    }

    /**
     * 更新滤波器并返回 R_level_lidar（从雷达坐标系到水平坐标系的旋转）
     *
     * 只补偿 roll / pitch，yaw 保持为零，保证航向不被改变。
     *
     * @param gravity_world  重力矢量在世界坐标系下的表示（来自 NavState::grav_）
     * @param R_world_lidar  雷达坐标系到世界坐标系的旋转矩阵 = rot_ * R_lidar_imu
     * @param timestamp      当前时间戳（秒）
     * @return SO3 旋转 R_level_lidar，使得 p_level = R_level_lidar * p_lidar
     */
    SO3 Update(const Vec3d& gravity_world, const Mat3d& R_world_lidar, double timestamp) {
        // 1. 将重力矢量从世界系变换到雷达系
        Vec3d g_lidar = R_world_lidar.transpose() * gravity_world;

        // 2. 低通滤波（在雷达系中滤波，有效消除步态频率的抖动）
        if (!initialized_) {
            g_lidar_filtered_ = g_lidar;
            last_timestamp_ = timestamp;
            initialized_ = true;
        } else {
            double dt = timestamp - last_timestamp_;
            if (dt > 0 && dt < 1.0) {
                double alpha = dt / (tau_ + dt);
                g_lidar_filtered_ = (1.0 - alpha) * g_lidar_filtered_ + alpha * g_lidar;
            }
            last_timestamp_ = timestamp;
        }

        // 3. 将滤波后的重力方向变换回世界系
        Vec3d g_world_filtered = R_world_lidar * g_lidar_filtered_;
        Vec3d world_up = -g_world_filtered.normalized();  // 重力反向 = "up"

        // 4. 构造水平坐标系（level_frame）的三个轴，全部在世界系中表示
        //
        // 关键：X 轴锚定到世界系 X 轴（与 map 对齐），而不是雷达系 X 轴。
        // 这样无论雷达安装偏角如何（含 90° yaw 偏装），level_frame 的航向
        // 始终与建图时的世界坐标系一致，地图与点云自然对齐。
        //
        //   Z_level = world_up（重力矫正，低通滤波后）
        //   X_level = 世界 X 轴投影到水平面（归一化）
        //   Y_level = Z_level × X_level

        Vec3d world_x(1.0, 0.0, 0.0);
        Vec3d level_x = world_x - world_x.dot(world_up) * world_up;
        if (level_x.norm() < 1e-6) {
            // 极端情况：世界 X 几乎垂直，改用世界 Y 轴
            level_x = Vec3d(0.0, 1.0, 0.0);
            level_x = level_x - level_x.dot(world_up) * world_up;
        }
        level_x.normalize();
        Vec3d level_y = world_up.cross(level_x);  // 右手系：Z × X = Y

        // 5. 构造 R_world_level（列向量为 level_frame 三轴在世界系中的方向）
        Mat3d R_world_level;
        R_world_level.col(0) = level_x;
        R_world_level.col(1) = level_y;
        R_world_level.col(2) = world_up;

        // 6. R_level_lidar = R_world_level^T * R_world_lidar
        //    R_world_lidar 由 LIO ESKF 迭代累积, 天然会偏离正交 (~1e-6); 再与 R_world_level
        //    (由归一化向量与叉乘拼出的列) 相乘会把漂移放大, 直接走 SO3(Matrix) 构造会触发
        //    Sophus 的 isOrthogonal 断言. 用 fitToSO3 做一次 SVD 正交化兜底.
        return SO3::fitToSO3(R_world_level.transpose() * R_world_lidar);
    }

    bool IsInitialized() const { return initialized_; }
    const Options& GetOptions() const { return options_; }

   private:
    Options options_;
    double tau_ = 0.318;  // 时间常数 1/(2*pi*f_c)
    Vec3d g_lidar_filtered_ = Vec3d(0.0, 0.0, -1.0);
    double last_timestamp_ = 0.0;
    bool initialized_ = false;
};

}  // namespace hikari::loclite

#endif  // HIKARI_LOCLITE_GRAVITY_ALIGNMENT_H
