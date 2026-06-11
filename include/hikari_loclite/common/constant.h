#pragma once

#ifndef HIKARI_LOCLITE_CONSTANT_H
#define HIKARI_LOCLITE_CONSTANT_H

#include "common/eigen_types.h"

namespace hikari::loclite::constant {

constexpr double kDEG2RAD = 0.017453292519943;
constexpr double kRAD2DEG = 57.295779513082323;
constexpr double kPI = M_PI;
constexpr double kPI_2 = kPI / 2.0;

constexpr double kGRAVITY = 9.80665;
constexpr double G_m_s2 = 9.806;

}  // namespace hikari::loclite::constant

#endif
