#pragma once

#ifndef HIKARI_LOCLITE_LOG_H
#define HIKARI_LOCLITE_LOG_H

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

}  // namespace hikari::loclite

#define LOG(severity) hikari::loclite::LogShim(#severity)

#define HIKARI_LOG_CONCAT_INNER(a, b) a##b
#define HIKARI_LOG_CONCAT(a, b) HIKARI_LOG_CONCAT_INNER(a, b)

#define LOG_EVERY_N(severity, n)                                      \
  static int HIKARI_LOG_CONCAT(_hikari_log_counter_, __LINE__) = 0;    \
  if (HIKARI_LOG_CONCAT(_hikari_log_counter_, __LINE__)++ % (n) == 0)  \
    hikari::loclite::LogShim(#severity)

#define DLOG(severity) hikari::loclite::LogShim(#severity)

#endif  // HIKARI_LOCLITE_LOG_H
