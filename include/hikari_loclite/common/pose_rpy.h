#pragma once

#ifndef HIKARI_LOCLITE_POSE_RPY_H
#define HIKARI_LOCLITE_POSE_RPY_H

namespace hikari::loclite {

template <typename T>
struct PoseRPY {
    PoseRPY() = default;
    PoseRPY(T xx, T yy, T zz, T r, T p, T y) : x(xx), y(yy), z(zz), roll(r), pitch(p), yaw(y) {}
    T x = 0;
    T y = 0;
    T z = 0;
    T roll = 0;
    T pitch = 0;
    T yaw = 0;
};

using PoseRPYD = PoseRPY<double>;
using PoseRPYF = PoseRPY<float>;

}  // namespace hikari::loclite

#endif
