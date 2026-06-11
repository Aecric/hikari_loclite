#pragma once

#ifndef HIKARI_LOCLITE_MEASURE_GROUP_H
#define HIKARI_LOCLITE_MEASURE_GROUP_H

#include <deque>

#include "common/imu.h"
#include "common/odom.h"
#include "common/point_def.h"

namespace hikari::loclite {

struct MeasureGroup {
    double timestamp_ = 0;
    double lidar_begin_time_ = 0;
    double lidar_end_time_ = 0;

    std::deque<IMUPtr> imu_;
    std::deque<OdomPtr> odom_;

    CloudPtr scan_raw_ = nullptr;
    CloudPtr scan_ = nullptr;
    CloudPtr scan_undist_ = nullptr;
    CloudPtr scan_undist_raw_ = nullptr;
};

}  // namespace hikari::loclite

#endif
