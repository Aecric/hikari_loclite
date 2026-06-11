#pragma once

#ifndef HIKARI_LOCLITE_NDT_CORRECTOR_HPP
#define HIKARI_LOCLITE_NDT_CORRECTOR_HPP

#include <memory>
#include <string>

#include "common/eigen_types.h"
#include "common/point_def.h"

namespace hikari::loclite {

struct NdtResult {
    bool valid = false;
    SE3 pose;
    double confidence = 0.0;
    double inlier_ratio = 0.0;
    double delta_trans_m = 0.0;
    double delta_rot_deg = 0.0;
};

class NdtCorrector {
   public:
    using Ptr = std::shared_ptr<NdtCorrector>;

    bool Init(const std::string& yaml_path);
    bool SetMap(const CloudPtr& map);

    NdtResult Align(const CloudPtr& scan, const SE3& guess);
    NdtResult Validate(const SE3& candidate_pose, const CloudPtr& scan);

   private:
    int threads_ = 1;
    double resolution_ = 1.0;
    double min_confidence_ = 1.0;
    double max_delta_trans_m_ = 1.0;
    double max_delta_rot_deg_ = 10.0;
};

}  // namespace hikari::loclite

#endif
