#include "lio/scan_context.h"
#include "log.h"
#include <iomanip>

namespace hikari::loclite {

float ScanContextManager::xy2theta(float x, float y) {
    if (x >= 0 && y >= 0) return (180.0f / M_PI) * atan2f(y, x);
    if (x < 0 && y >= 0) return 180.0f - (180.0f / M_PI) * atan2f(y, -x);
    if (x < 0 && y < 0) return 180.0f + (180.0f / M_PI) * atan2f(-y, -x);
    return 360.0f - (180.0f / M_PI) * atan2f(-y, x);
}

Eigen::MatrixXd ScanContextManager::circshift(Eigen::MatrixXd& mat, int num_shift) {
    if (num_shift == 0) return Eigen::MatrixXd(mat);
    Eigen::MatrixXd shifted_mat = Eigen::MatrixXd::Zero(mat.rows(), mat.cols());
    for (int col_idx = 0; col_idx < mat.cols(); col_idx++) {
        int new_location = (col_idx + num_shift) % mat.cols();
        shifted_mat.col(new_location) = mat.col(col_idx);
    }
    return shifted_mat;
}

std::vector<float> ScanContextManager::eig2stdvec(Eigen::MatrixXd eigmat) {
    std::vector<float> vec(eigmat.data(), eigmat.data() + eigmat.size());
    return vec;
}

Eigen::MatrixXd ScanContextManager::makeScancontext(const CloudPtr& cloud) {
    const int NO_POINT = -1000;
    Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Ones(options_.pc_num_ring, options_.pc_num_sector);

    for (size_t pt_idx = 0; pt_idx < cloud->points.size(); pt_idx++) {
        float pt_x = cloud->points[pt_idx].x;
        float pt_y = cloud->points[pt_idx].y;
        float pt_z = cloud->points[pt_idx].z + options_.lidar_height;
        float azim_range = sqrtf(pt_x * pt_x + pt_y * pt_y);
        float azim_angle = xy2theta(pt_x, pt_y);
        if (azim_range > options_.pc_max_radius) continue;

        int ring_idx = std::max(std::min(options_.pc_num_ring, int(ceil((azim_range / options_.pc_max_radius) * options_.pc_num_ring))), 1);
        int sector_idx = std::max(std::min(options_.pc_num_sector, int(ceil((azim_angle / 360.0) * options_.pc_num_sector))), 1);

        if (desc(ring_idx - 1, sector_idx - 1) < pt_z) desc(ring_idx - 1, sector_idx - 1) = pt_z;
    }

    for (int row_idx = 0; row_idx < desc.rows(); row_idx++)
        for (int col_idx = 0; col_idx < desc.cols(); col_idx++)
            if (desc(row_idx, col_idx) == NO_POINT) desc(row_idx, col_idx) = 0;

    return desc;
}

Eigen::MatrixXd ScanContextManager::makeRingkeyFromScancontext(Eigen::MatrixXd& desc) {
    Eigen::MatrixXd invariant_key(desc.rows(), 1);
    for (int row_idx = 0; row_idx < desc.rows(); row_idx++) {
        invariant_key(row_idx, 0) = desc.row(row_idx).mean();
    }
    return invariant_key;
}

Eigen::MatrixXd ScanContextManager::makeSectorkeyFromScancontext(Eigen::MatrixXd& desc) {
    Eigen::MatrixXd variant_key(1, desc.cols());
    for (int col_idx = 0; col_idx < desc.cols(); col_idx++) {
        variant_key(0, col_idx) = desc.col(col_idx).mean();
    }
    return variant_key;
}

double ScanContextManager::distDirectSC(Eigen::MatrixXd& sc1, Eigen::MatrixXd& sc2) {
    int num_eff_cols = 0;
    double sum_sector_similarity = 0;
    for (int col_idx = 0; col_idx < sc1.cols(); col_idx++) {
        Eigen::VectorXd col_sc1 = sc1.col(col_idx);
        Eigen::VectorXd col_sc2 = sc2.col(col_idx);
        if (col_sc1.norm() == 0 || col_sc2.norm() == 0) continue;
        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());
        sum_sector_similarity += sector_similarity;
        num_eff_cols++;
    }
    if (num_eff_cols == 0) return 1.0;
    return 1.0 - sum_sector_similarity / num_eff_cols;
}

int ScanContextManager::fastAlignUsingVkey(Eigen::MatrixXd& vkey1, Eigen::MatrixXd& vkey2) {
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 1e10;
    for (int shift_idx = 0; shift_idx < vkey1.cols(); shift_idx++) {
        Eigen::MatrixXd vkey2_shifted = circshift(vkey2, shift_idx);
        Eigen::MatrixXd vkey_diff = vkey1 - vkey2_shifted;
        double cur_diff_norm = vkey_diff.norm();
        if (cur_diff_norm < min_veky_diff_norm) {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }
    return argmin_vkey_shift;
}

std::pair<double, int> ScanContextManager::distanceBtnScanContext(Eigen::MatrixXd& sc1, Eigen::MatrixXd& sc2) {
    Eigen::MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(sc1);
    Eigen::MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(sc2);
    int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

    const int SEARCH_RADIUS = round(0.5 * options_.search_ratio * sc1.cols());
    std::vector<int> shift_idx_search_space{argmin_vkey_shift};
    for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++) {
        shift_idx_search_space.push_back((argmin_vkey_shift + ii + sc1.cols()) % sc1.cols());
        shift_idx_search_space.push_back((argmin_vkey_shift - ii + sc1.cols()) % sc1.cols());
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    int argmin_shift = 0;
    double min_sc_dist = 1e10;
    for (int num_shift : shift_idx_search_space) {
        Eigen::MatrixXd sc2_shifted = circshift(sc2, num_shift);
        double cur_sc_dist = distDirectSC(sc1, sc2_shifted);
        if (cur_sc_dist < min_sc_dist) {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }
    return std::make_pair(min_sc_dist, argmin_shift);
}

void ScanContextManager::makeAndSaveScancontextAndKeys(const CloudPtr& cloud, int kf_id) {
    if (!cloud || cloud->empty()) return;
    Eigen::MatrixXd sc = makeScancontext(cloud);
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc);
    std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);

    std::lock_guard<std::mutex> lock(data_mutex_);
    polarcontexts_.push_back(sc);
    polarcontext_invkeys_.push_back(ringkey);
    polarcontext_vkeys_.push_back(sectorkey);
    polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);
    kf_ids_.push_back(kf_id);
}

std::pair<int, float> ScanContextManager::Query(const CloudPtr& cloud) {
    if (!cloud || cloud->empty()) return {-1, 0.0f};
    Eigen::MatrixXd sc = makeScancontext(cloud);
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
    std::vector<float> curr_key = eig2stdvec(ringkey);

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (polarcontexts_.empty()) return {-1, 0.0f};

    if (!polarcontext_tree_ || polarcontext_invkeys_to_search_.size() != polarcontext_invkeys_mat_.size()) {
        polarcontext_invkeys_to_search_ = polarcontext_invkeys_mat_;
        polarcontext_tree_ = std::make_unique<InvKeyTree>(options_.pc_num_ring, polarcontext_invkeys_to_search_, 10);
    }

    double min_dist = 1e10;
    int nn_align = 0;
    int nn_idx = -1;

    int num_candidates = std::min(options_.num_candidates_from_tree, (int)polarcontext_invkeys_to_search_.size());
    if (num_candidates == 0) return {-1, 0.0f};

    std::vector<size_t> candidate_indexes(num_candidates);
    std::vector<float> out_dists_sqr(num_candidates);
    nanoflann::KNNResultSet<float> knnsearch_result(num_candidates);
    knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
    polarcontext_tree_->index->findNeighbors(knnsearch_result, &curr_key[0], nanoflann::SearchParams(10));

    for (int i = 0; i < num_candidates; i++) {
        Eigen::MatrixXd candidate_sc = polarcontexts_[candidate_indexes[i]];
        auto sc_dist_result = distanceBtnScanContext(sc, candidate_sc);
        if (sc_dist_result.first < min_dist) {
            min_dist = sc_dist_result.first;
            nn_align = sc_dist_result.second;
            nn_idx = candidate_indexes[i];
        }
    }

    if (min_dist < options_.sc_dist_thres) {
        const int real_kf_id = (nn_idx >= 0 && nn_idx < (int)kf_ids_.size()) ? kf_ids_[nn_idx] : nn_idx;
        float yaw_diff_rad = nn_align * pc_unit_sector_angle_ * M_PI / 180.0;
        return {real_kf_id, yaw_diff_rad};
    }
    return {-1, 0.0f};
}

std::vector<ScanContextManager::Candidate> ScanContextManager::QueryTopK(const CloudPtr& cloud, int k) {
    std::vector<Candidate> result;
    if (k <= 0 || !cloud || cloud->empty()) return result;

    Eigen::MatrixXd sc = makeScancontext(cloud);
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
    std::vector<float> curr_key = eig2stdvec(ringkey);

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (polarcontexts_.empty()) return result;

    if (!polarcontext_tree_ || polarcontext_invkeys_to_search_.size() != polarcontext_invkeys_mat_.size()) {
        polarcontext_invkeys_to_search_ = polarcontext_invkeys_mat_;
        polarcontext_tree_ = std::make_unique<InvKeyTree>(options_.pc_num_ring, polarcontext_invkeys_to_search_, 10);
    }

    int pool_size = std::max(k, std::max(options_.num_candidates_from_tree, 100));
    pool_size = std::min(pool_size, (int)polarcontext_invkeys_to_search_.size());
    if (pool_size == 0) return result;

    std::vector<size_t> candidate_indexes(pool_size);
    std::vector<float> out_dists_sqr(pool_size);
    nanoflann::KNNResultSet<float> knnsearch_result(pool_size);
    knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
    polarcontext_tree_->index->findNeighbors(knnsearch_result, &curr_key[0], nanoflann::SearchParams(10));

    std::vector<Candidate> scored;
    scored.reserve(pool_size);
    for (int i = 0; i < pool_size; i++) {
        Eigen::MatrixXd candidate_sc = polarcontexts_[candidate_indexes[i]];
        auto sc_dist_result = distanceBtnScanContext(sc, candidate_sc);
        Candidate c;
        c.kf_id = kf_ids_[static_cast<int>(candidate_indexes[i])];
        c.sc_dist = sc_dist_result.first;
        c.yaw_diff_rad = static_cast<float>(sc_dist_result.second * pc_unit_sector_angle_ * M_PI / 180.0);
        scored.push_back(c);
    }
    std::sort(scored.begin(), scored.end(), [](const Candidate& a, const Candidate& b) { return a.sc_dist < b.sc_dist; });

    for (const auto& c : scored) {
        if (c.sc_dist >= options_.sc_dist_thres) break;
        result.push_back(c);
        if ((int)result.size() >= k) break;
    }
    return result;
}

static constexpr uint32_t kScDbMagic = 0x53434442;
static constexpr uint32_t kScDbVersion = 2;

bool ScanContextManager::SaveDatabase(const std::string& path) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open()) return false;

    ofs.write(reinterpret_cast<const char*>(&kScDbMagic), sizeof(uint32_t));
    ofs.write(reinterpret_cast<const char*>(&kScDbVersion), sizeof(uint32_t));

    uint64_t num_entries = polarcontexts_.size();
    int32_t num_ring = options_.pc_num_ring;
    int32_t num_sector = options_.pc_num_sector;
    ofs.write(reinterpret_cast<const char*>(&num_entries), sizeof(uint64_t));
    ofs.write(reinterpret_cast<const char*>(&num_ring), sizeof(int32_t));
    ofs.write(reinterpret_cast<const char*>(&num_sector), sizeof(int32_t));

    for (uint64_t i = 0; i < num_entries; i++) {
        ofs.write(reinterpret_cast<const char*>(polarcontexts_[i].data()), polarcontexts_[i].size() * sizeof(double));
        ofs.write(reinterpret_cast<const char*>(polarcontext_invkeys_[i].data()), polarcontext_invkeys_[i].size() * sizeof(double));
        ofs.write(reinterpret_cast<const char*>(polarcontext_vkeys_[i].data()), polarcontext_vkeys_[i].size() * sizeof(double));
        ofs.write(reinterpret_cast<const char*>(polarcontext_invkeys_mat_[i].data()), polarcontext_invkeys_mat_[i].size() * sizeof(float));
        int32_t kid = kf_ids_[i];
        ofs.write(reinterpret_cast<const char*>(&kid), sizeof(int32_t));
    }
    return ofs.good();
}

bool ScanContextManager::LoadDatabase(const std::string& path) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) return false;

    uint32_t magic = 0, version = 0;
    ifs.read(reinterpret_cast<char*>(&magic), sizeof(uint32_t));
    ifs.read(reinterpret_cast<char*>(&version), sizeof(uint32_t));
    if (magic != kScDbMagic || version != kScDbVersion) return false;

    uint64_t num_entries = 0;
    int32_t num_ring = 0, num_sector = 0;
    ifs.read(reinterpret_cast<char*>(&num_entries), sizeof(uint64_t));
    ifs.read(reinterpret_cast<char*>(&num_ring), sizeof(int32_t));
    ifs.read(reinterpret_cast<char*>(&num_sector), sizeof(int32_t));
    if (!ifs.good()) return false;

    std::vector<Eigen::MatrixXd> tmp_polarcontexts, tmp_invkeys, tmp_vkeys;
    std::vector<std::vector<float>> tmp_invkeys_mat;
    std::vector<int> tmp_kf_ids;
    tmp_polarcontexts.reserve(num_entries);
    tmp_invkeys.reserve(num_entries);
    tmp_vkeys.reserve(num_entries);
    tmp_invkeys_mat.reserve(num_entries);
    tmp_kf_ids.reserve(num_entries);

    for (uint64_t i = 0; i < num_entries; i++) {
        Eigen::MatrixXd sc(num_ring, num_sector);
        ifs.read(reinterpret_cast<char*>(sc.data()), sc.size() * sizeof(double));
        if (!ifs.good()) return false;
        tmp_polarcontexts.push_back(sc);

        Eigen::MatrixXd rk(num_ring, 1);
        ifs.read(reinterpret_cast<char*>(rk.data()), rk.size() * sizeof(double));
        if (!ifs.good()) return false;
        tmp_invkeys.push_back(rk);

        Eigen::MatrixXd sk(1, num_sector);
        ifs.read(reinterpret_cast<char*>(sk.data()), sk.size() * sizeof(double));
        if (!ifs.good()) return false;
        tmp_vkeys.push_back(sk);

        std::vector<float> rk_vec(num_ring);
        ifs.read(reinterpret_cast<char*>(rk_vec.data()), rk_vec.size() * sizeof(float));
        if (!ifs.good()) return false;
        tmp_invkeys_mat.push_back(rk_vec);

        int32_t kid = -1;
        ifs.read(reinterpret_cast<char*>(&kid), sizeof(int32_t));
        if (!ifs.good()) return false;
        tmp_kf_ids.push_back(kid);
    }

    polarcontexts_.swap(tmp_polarcontexts);
    polarcontext_invkeys_.swap(tmp_invkeys);
    polarcontext_vkeys_.swap(tmp_vkeys);
    polarcontext_invkeys_mat_.swap(tmp_invkeys_mat);
    kf_ids_.swap(tmp_kf_ids);
    polarcontext_tree_.reset();
    polarcontext_invkeys_to_search_.clear();

    options_.pc_num_ring = num_ring;
    options_.pc_num_sector = num_sector;
    UpdateDerivedParams();

    LOG(INFO) << "ScanContext: loaded database (" << num_entries << " entries) from " << path;
    return true;
}

}  // namespace hikari::loclite
