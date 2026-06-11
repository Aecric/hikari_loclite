#pragma once

#ifndef HIKARI_LOCLITE_OPTIONS_H
#define HIKARI_LOCLITE_OPTIONS_H

#include <string>

#include <common/constant.h>
#include <common/eigen_types.h>

namespace hikari::loclite {

namespace fasterlio {

constexpr double INIT_TIME = 0.1;
constexpr int NUM_MATCH_POINTS = 5;
constexpr int MIN_NUM_MATCH_POINTS = 3;

constexpr int NUM_MAX_ITERATIONS = 4;
constexpr float ESTI_PLANE_THRESHOLD = 0.1f;

}  // namespace fasterlio

namespace lo {

inline float lidar_time_interval = 0.1f;

}  // namespace lo

}  // namespace hikari::loclite

#endif
