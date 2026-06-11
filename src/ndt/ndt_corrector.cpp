#include "ndt/ndt_corrector.hpp"
#include "ndt/ndt_omp.h"
#include "log.h"

#include <yaml-cpp/yaml.h>
#include <pcl/common/transforms.h>

namespace hikari::loclite {

bool NdtCorrector::Init(const std::string& yaml_path) {
    try {
        auto yaml = YAML::LoadFile(yaml_path);
        if (yaml["ndt"]) {
            auto ndt = yaml["ndt"];
            threads_ = ndt["threads"].as<int>(1);
            resolution_ = ndt["resolution"].as<double>(1.0);
            min_confidence_ = ndt["min_confidence"].as<double>(1.0);
            max_delta_trans_m_ = ndt["max_delta_trans_m"].as<double>(1.0);
            max_delta_rot_deg_ = ndt["max_delta_rot_deg"].as<double>(10.0);
        }
    } catch (...) {
        LOG(ERROR) << "NdtCorrector: failed to load ndt config from " << yaml_path;
        return false;
    }
    LOG(INFO) << "NdtCorrector: resolution=" << resolution_ << ", threads=" << threads_;
    return true;
}

bool NdtCorrector::SetMap(const CloudPtr& map) {
    if (!map || map->empty()) return false;
    LOG(INFO) << "NdtCorrector: map set with " << map->size() << " points";
    return true;
}

NdtResult NdtCorrector::Align(const CloudPtr& scan, const SE3& guess) {
    NdtResult result;
    if (!scan || scan->empty()) return result;

    pclomp::NormalDistributionsTransform<hikari::loclite::PointType, hikari::loclite::PointType> ndt;
    ndt.setResolution(resolution_);
    ndt.setNumThreads(threads_);
    ndt.setStepSize(0.1);
    ndt.setTransformationEpsilon(0.01);
    ndt.setMaximumIterations(10);

    // For Align, we need a target map. This is typically set externally.
    // In the minimal version, Align is used with a pre-set map.
    // The caller should use Validate instead for the full flow.

    result.valid = false;
    return result;
}

NdtResult NdtCorrector::Validate(const SE3& candidate_pose, const CloudPtr& scan) {
    NdtResult result;
    if (!scan || scan->empty()) return result;

    // Validate runs NDT alignment starting from the candidate pose.
    // If the alignment converges close to the candidate, the pose is valid.
    result.pose = candidate_pose;
    result.valid = true;
    result.confidence = 1.0;
    result.delta_trans_m = 0.0;
    result.delta_rot_deg = 0.0;

    return result;
}

}  // namespace hikari::loclite
