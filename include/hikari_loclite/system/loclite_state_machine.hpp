#pragma once

#ifndef HIKARI_LOCLITE_STATE_MACHINE_HPP
#define HIKARI_LOCLITE_STATE_MACHINE_HPP

#include <string>

namespace hikari::loclite {

enum class LiteLocState {
    Uninitialized = 0,
    Initializing = 1,
    Good = 2,
    Degraded = 3,
    Lost = 4,
    WaitForInitialPose = 5,
};

class LocLiteStateMachine {
   public:
    LiteLocState State() const { return state_; }
    const char* StateStr() const {
        switch (state_) {
            case LiteLocState::Uninitialized: return "Uninitialized";
            case LiteLocState::Initializing: return "Initializing";
            case LiteLocState::Good: return "Good";
            case LiteLocState::Degraded: return "Degraded";
            case LiteLocState::Lost: return "Lost";
            case LiteLocState::WaitForInitialPose: return "WaitForInitialPose";
        }
        return "Unknown";
    }

    void SetInitializing(const char* reason) {
        // /initialpose 注入时的状态机原子复位: bad/good 计数一并清零
        state_ = LiteLocState::Initializing;
        bad_count_ = 0;
        good_count_ = 0;
        reason_ = reason ? reason : "";
    }

    void SetGood(const char* reason) {
        state_ = LiteLocState::Good;
        bad_count_ = 0;
        good_count_ = 0;
        reason_ = reason ? reason : "";
    }

    void SetLost(const char* reason) {
        state_ = LiteLocState::Lost;
        reason_ = reason ? reason : "";
    }

    void SetWaitForInitialPose(const char* reason) {
        state_ = LiteLocState::WaitForInitialPose;
        reason_ = reason ? reason : "";
    }

    void ObserveTrackingQuality(bool good) {
        if (state_ != LiteLocState::Good && state_ != LiteLocState::Degraded) return;
        if (good) {
            if (state_ == LiteLocState::Degraded && ++good_count_ >= recover_good_frames_) {
                SetGood("quality_recovered");
            }
            bad_count_ = 0;
            return;
        }
        good_count_ = 0;
        ++bad_count_;
        if (bad_count_ >= lost_bad_frames_) {
            state_ = LiteLocState::Lost;
            reason_ = "quality_lost";
        } else if (bad_count_ >= degraded_bad_frames_) {
            state_ = LiteLocState::Degraded;
            reason_ = "quality_degraded";
        }
    }

    const std::string& Reason() const { return reason_; }

   private:
    LiteLocState state_ = LiteLocState::Uninitialized;
    std::string reason_;
    int bad_count_ = 0;
    int good_count_ = 0;
    int degraded_bad_frames_ = 3;
    int lost_bad_frames_ = 10;
    int recover_good_frames_ = 5;
};

}  // namespace hikari::loclite

#endif
