#pragma once

#ifndef HIKARI_LOCLITE_MATH_HPP
#define HIKARI_LOCLITE_MATH_HPP

#include <Eigen/Dense>
#include <cmath>
#include <numeric>
#include <pcl/filters/voxel_grid.h>
#include <vector>

#include "common/eigen_types.h"
#include "common/options.h"
#include "common/point_def.h"

namespace hikari::loclite::math {

// ==================== SO3 utilities ====================

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const Eigen::Matrix<T, 3, 1>& v) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
    return m;
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> SKEW_SYM_MATRIX(const T& v1, const T& v2, const T& v3) {
    Eigen::Matrix<T, 3, 3> m;
    m << 0.0, -v3, v2, v3, 0.0, -v1, -v2, v1, 0.0;
    return m;
}

template <typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1>& ang_vel, const Ts& dt) {
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_vel_norm > 0.0000001) {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K = SKEW_SYM_MATRIX(r_axis);
        T r_ang = ang_vel_norm * dt;
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    return Eye3;
}

/// SO3 exp with scale (cos_sinc_sqrt based, numerically stable)
template <class scalar>
inline std::pair<scalar, scalar> cos_sinc_sqrt(const scalar& x2) {
    using std::cos;
    using std::sin;
    using std::sqrt;
    static scalar const taylor_0_bound = std::numeric_limits<scalar>::epsilon();
    static scalar const taylor_2_bound = sqrt(taylor_0_bound);
    static scalar const taylor_n_bound = sqrt(taylor_2_bound);

    if (x2 >= taylor_n_bound) {
        scalar x = sqrt(x2);
        return std::make_pair(cos(x), sin(x) / x);
    }

    static scalar const inv[] = {1 / 3., 1 / 4., 1 / 5., 1 / 6., 1 / 7., 1 / 8., 1 / 9.};
    scalar cosi = 1., sinc = 1;
    scalar term = -1 / 2. * x2;
    for (int i = 0; i < 3; ++i) {
        cosi += term;
        term *= inv[2 * i];
        sinc += term;
        term *= -inv[2 * i + 1] * x2;
    }
    return std::make_pair(cosi, sinc);
}

inline SO3 exp(const Vec3d& vec, const double& scale = 1) {
    double norm2 = vec.squaredNorm();
    std::pair<double, double> cos_sinc = cos_sinc_sqrt(scale * scale * norm2);
    double mult = cos_sinc.second * scale;
    Vec3d result = mult * vec;
    return SO3(Quatd(cos_sinc.first, result[0], result[1], result[2]));
}

/// SO3 Jl()/JacobianL()
inline Eigen::Matrix<double, 3, 3> A_matrix(const Vec3d& v) {
    Eigen::Matrix<double, 3, 3> res;
    double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    double norm = std::sqrt(squaredNorm);
    if (norm < 1e-5) {
        res = Eigen::Matrix<double, 3, 3>::Identity();
    } else {
        res = Eigen::Matrix<double, 3, 3>::Identity() + (1 - std::cos(norm)) / squaredNorm * SO3::hat(v) +
              (1 - std::sin(norm) / norm) / squaredNorm * SO3::hat(v) * SO3::hat(v);
    }
    return res;
}

// ==================== Hash / comparison ====================

template <int N>
struct hash_vec {
    inline size_t operator()(const Eigen::Matrix<int, N, 1>& v) const;
};

template <>
inline size_t hash_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v) const {
    return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943)) % 10000000;
}

template <>
inline size_t hash_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v) const {
    return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^ ((v[2]) * 83492791)) % 10000000;
}

template <int N>
struct less_vec {
    inline bool operator()(const Eigen::Matrix<int, N, 1>& v1, const Eigen::Matrix<int, N, 1>& v2) const;
};

template <>
inline bool less_vec<2>::operator()(const Eigen::Matrix<int, 2, 1>& v1, const Eigen::Matrix<int, 2, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]);
}

template <>
inline bool less_vec<3>::operator()(const Eigen::Matrix<int, 3, 1>& v1, const Eigen::Matrix<int, 3, 1>& v2) const {
    return v1[0] < v2[0] || (v1[0] == v2[0] && v1[1] < v2[1]) || (v1[0] == v2[0] && v1[1] == v2[1] && v1[2] < v2[2]);
}

// ==================== Plane estimation ====================

template <typename T>
inline bool esti_plane(Eigen::Matrix<T, 4, 1>& pca_result, const PointVector& point, const T& threshold = 0.1f) {
    if (point.size() < fasterlio::MIN_NUM_MATCH_POINTS) {
        return false;
    }

    Eigen::Matrix<T, 3, 1> normvec;

    if (point.size() == fasterlio::NUM_MATCH_POINTS) {
        Eigen::Matrix<T, fasterlio::NUM_MATCH_POINTS, 3> A;
        Eigen::Matrix<T, fasterlio::NUM_MATCH_POINTS, 1> b;
        A.setZero();
        b.setOnes();
        b *= -1.0f;
        for (int j = 0; j < fasterlio::NUM_MATCH_POINTS; j++) {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }
        normvec = A.colPivHouseholderQr().solve(b);
    } else {
        Eigen::MatrixXd A(point.size(), 3);
        Eigen::VectorXd b(point.size(), 1);
        A.setZero();
        b.setOnes();
        b *= -1.0f;
        for (int j = 0; j < (int)point.size(); j++) {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }
        Eigen::MatrixXd n = A.colPivHouseholderQr().solve(b);
        normvec(0, 0) = n(0, 0);
        normvec(1, 0) = n(1, 0);
        normvec(2, 0) = n(2, 0);
    }

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (const auto& p : point) {
        Eigen::Matrix<T, 4, 1> temp = p.getVector4fMap();
        temp[3] = 1.0;
        if (fabs(pca_result.dot(temp)) > threshold) {
            return false;
        }
    }
    return true;
}

// ==================== Distance ====================

inline float calc_dist(const PointType& p1, const PointType& p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
}

inline float calc_dist(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) { return (p1 - p2).squaredNorm(); }

template <typename PointT>
inline double distance2(const PointT& pt1, const PointT& pt2) {
    Eigen::Vector3f d = pt1.getVector3fMap() - pt2.getVector3fMap();
    return d.squaredNorm();
}

// ==================== Array conversion ====================

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<double>& v) {
    return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<double>& v) {
    Eigen::Matrix<S, 3, 3> m;
    m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return m;
}

// ==================== Misc ====================

inline CloudPtr VoxelGrid(CloudPtr cloud, float voxel_size = 0.05) {
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel.setInputCloud(cloud);
    CloudPtr output(new PointCloudType);
    voxel.filter(*output);
    return output;
}

inline double ToSec(uint64_t t) { return double(t) * 1e-9; }

template <typename T, int dim, typename PointT>
inline Eigen::Matrix<T, dim, 1> ToEigen(const PointT& pt) {
    return Eigen::Matrix<T, dim, 1>(pt.x, pt.y, pt.z);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZ>(const pcl::PointXYZ& pt) {
    return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 2, 1> ToEigen<float, 2, PointXYZIT>(const PointXYZIT& pt) {
    return Vec2f(pt.x, pt.y);
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZI>(const pcl::PointXYZI& pt) {
    return pt.getVector3fMap();
}

template <>
inline Eigen::Matrix<float, 3, 1> ToEigen<float, 3, pcl::PointXYZINormal>(const pcl::PointXYZINormal& pt) {
    return pt.getVector3fMap();
}

template <typename T>
T rad2deg(const T& radians) { return radians * 180.0 / M_PI; }

template <typename T>
T deg2rad(const T& degrees) { return degrees * M_PI / 180.0; }

}  // namespace hikari::loclite::math

#endif
