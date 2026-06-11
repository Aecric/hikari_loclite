#pragma once

#ifndef HIKARI_LOCLITE_ODOM_H
#define HIKARI_LOCLITE_ODOM_H

#include "common/eigen_types.h"

namespace hikari::loclite {

struct Odom {
    double timestamp_ = 0;
    SE3 pose;
    Vec3d linear;
    Vec3d angular;
};

using OdomPtr = std::shared_ptr<Odom>;

}  // namespace hikari::loclite

#endif
