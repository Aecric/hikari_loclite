#pragma once

#ifndef HIKARI_LOCLITE_SCAN_CONTEXT_H
#define HIKARI_LOCLITE_SCAN_CONTEXT_H

#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>
#include <string>
#include <fstream>
#include <mutex>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include "common/point_def.h"

namespace hikari::loclite {

using KeyMat = std::vector<std::vector<float>>;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

class ScanContextManager {
   public:
    struct Options {
        double lidar_height = 2.0;
        double pc_max_radius = 80.0;
        int pc_num_ring = 20;
        int pc_num_sector = 60;
        int num_exclude_recent = 50;
        int num_candidates_from_tree = 10;
        double search_ratio = 0.1;
        double sc_dist_thres = 0.13;
    };

    ScanContextManager() = default;
    explicit ScanContextManager(const Options& options) : options_(options) { UpdateDerivedParams(); }

    void SetOptions(const Options& options) { options_ = options; UpdateDerivedParams(); }

    void makeAndSaveScancontextAndKeys(const CloudPtr& cloud, int kf_id);

    std::pair<int, float> Query(const CloudPtr& cloud);

    struct Candidate {
        int kf_id = -1;
        double sc_dist = 0.0;
        float yaw_diff_rad = 0.0f;
    };
    std::vector<Candidate> QueryTopK(const CloudPtr& cloud, int k);

    bool SaveDatabase(const std::string& path) const;
    bool LoadDatabase(const std::string& path);

    size_t GetScanContextCount() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return polarcontexts_.size();
    }

    int GetKfIdByIndex(int idx) const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (idx < 0 || idx >= (int)kf_ids_.size()) return -1;
        return kf_ids_[idx];
    }

   private:
    void UpdateDerivedParams() {
        pc_unit_sector_angle_ = 360.0 / double(options_.pc_num_sector);
        pc_unit_ring_gap_ = options_.pc_max_radius / double(options_.pc_num_ring);
    }

    Eigen::MatrixXd makeScancontext(const CloudPtr& cloud);
    Eigen::MatrixXd makeRingkeyFromScancontext(Eigen::MatrixXd& desc);
    Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd& desc);
    double distDirectSC(Eigen::MatrixXd& sc1, Eigen::MatrixXd& sc2);
    std::pair<double, int> distanceBtnScanContext(Eigen::MatrixXd& sc1, Eigen::MatrixXd& sc2);
    int fastAlignUsingVkey(Eigen::MatrixXd& vkey1, Eigen::MatrixXd& vkey2);

    static float xy2theta(float x, float y);
    static Eigen::MatrixXd circshift(Eigen::MatrixXd& mat, int num_shift);
    static std::vector<float> eig2stdvec(Eigen::MatrixXd eigmat);

    Options options_;
    double pc_unit_sector_angle_ = 360.0 / 60.0;
    double pc_unit_ring_gap_ = 80.0 / 20.0;

    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;
    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;
    std::vector<int> kf_ids_;
    mutable std::mutex data_mutex_;
};

}  // namespace hikari::loclite

#endif
