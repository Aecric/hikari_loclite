#pragma once

#ifndef HIKARI_LOCLITE_IMU_H
#define HIKARI_LOCLITE_IMU_H

#include "common/eigen_types.h"

namespace hikari::loclite {

struct IMU {
    double timestamp = 0;
    Vec3d angular_velocity = Vec3d::Zero();
    Vec3d linear_acceleration = Vec3d::Zero();
};

using IMUPtr = std::shared_ptr<IMU>;

}  // namespace hikari::loclite

#endif
