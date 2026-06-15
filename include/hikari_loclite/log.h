#pragma once

#ifndef HIKARI_LOCLITE_LOG_H
#define HIKARI_LOCLITE_LOG_H

#include <chrono>
#include <iostream>
#include <sstream>

namespace hikari::loclite {

/// Minimal logging shim replacing glog LOG() with std::cerr stream syntax.
class LogShim {
 public:
  explicit LogShim(const char* level) { ss_ << "[" << level << "] "; }
  ~LogShim() { std::cerr << ss_.str() << std::endl; }

  template <typename T>
  LogShim& operator<<(const T& v) {
    ss_ << v;
    return *this;
  }

  // Support Eigen types via operator<<
  LogShim& operator<<(const std::string& v) {
    ss_ << v;
    return *this;
  }

 private:
  std::ostringstream ss_;
};

/// No-op logger for LOG_EVERY_N when count doesn't match.
class NullLogShim {
 public:
  template <typename T>
  NullLogShim& operator<<(const T&) {
    return *this;
  }
};

/// 稳态单调时钟秒数, 供 LOG_EVERY_T 时间节流使用 (与传感器/bag 时间戳无关).
inline double SteadyNowSec() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// 距上次放行已超过 period 秒则返回 true 并更新 next; 否则 false. 仅用于日志限频, 容忍并发竞争.
inline bool LogEveryTReady(double& next, double period) {
  const double now = SteadyNowSec();
  if (now >= next) {
    next = now + period;
    return true;
  }
  return false;
}

}  // namespace hikari::loclite

#define LOG(severity) hikari::loclite::LogShim(#severity)

#define HIKARI_LOG_CONCAT_INNER(a, b) a##b
#define HIKARI_LOG_CONCAT(a, b) HIKARI_LOG_CONCAT_INNER(a, b)

#define LOG_EVERY_N(severity, n)                                      \
  static int HIKARI_LOG_CONCAT(_hikari_log_counter_, __LINE__) = 0;    \
  if (HIKARI_LOG_CONCAT(_hikari_log_counter_, __LINE__)++ % (n) == 0)  \
    hikari::loclite::LogShim(#severity)

// 时间式限频: 每 period_sec 秒最多放行一次 (按调用点 __LINE__ 各自计时).
// 热路径里输出帧率与帧率解耦的稳态诊断时优先用它, 比 LOG_EVERY_N 更贴"每秒几行"的预算.
#define LOG_EVERY_T(severity, period_sec)                                 \
  static double HIKARI_LOG_CONCAT(_hikari_log_next_t_, __LINE__) = 0.0;    \
  if (hikari::loclite::LogEveryTReady(                                     \
          HIKARI_LOG_CONCAT(_hikari_log_next_t_, __LINE__), (period_sec))) \
    hikari::loclite::LogShim(#severity)

#define DLOG(severity) hikari::loclite::LogShim(#severity)

#endif  // HIKARI_LOCLITE_LOG_H
