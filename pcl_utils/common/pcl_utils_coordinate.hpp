//
// Created by mfy on 2025/12/31.
//

#ifndef VISIONUTILS_PCL_UTILS_COORDINATE_HPP
#define VISIONUTILS_PCL_UTILS_COORDINATE_HPP

#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>

// 构造当前坐标系的旋转基
Eigen::Matrix3f buildCurrentBasis(const Eigen::Vector3f& up, const Eigen::Vector3f& forward)
{
    Eigen::Vector3f up_norm = up.normalized();
    Eigen::Vector3f forward_norm = forward.normalized();

    Eigen::Vector3f right = forward_norm.cross(up_norm);
    if (right.norm() < 1e-6)
        throw std::runtime_error("up and forward directions are parallel or invalid.");

    right.normalize();

    Eigen::Vector3f forward_ortho = up_norm.cross(right);
    forward_ortho.normalize();

    Eigen::Matrix3f basis;
    basis.col(0) = right;         // 当前坐标系的右轴
    basis.col(1) = forward_ortho; // 当前坐标系的前轴
    basis.col(2) = up_norm;       // 当前坐标系的上轴

    return basis;
}

// 目标坐标系基（固定）
Eigen::Matrix3f buildTargetBasis()
{
    Eigen::Matrix3f basis;
    basis.col(0) = Eigen::Vector3f(1,0,0); // 右X+
    basis.col(1) = Eigen::Vector3f(0,1,0); // 前Y+
    basis.col(2) = Eigen::Vector3f(0,0,1); // 上Z+
    return basis;
}

// 计算从当前坐标系到目标坐标系的旋转矩阵
Eigen::Matrix3f computeTransform(
    const Eigen::Vector3f& up_current,
    const Eigen::Vector3f& forward_current)
{
    Eigen::Matrix3f current_basis = buildCurrentBasis(up_current, forward_current);
    Eigen::Matrix3f target_basis = buildTargetBasis();

    // 当前坐标系基到世界： current_basis
    // 目标坐标系基到世界： target_basis
    // 点云从当前系变到目标系的矩阵是：
    // R = target_basis * current_basis.transpose()
    return target_basis * current_basis.transpose();
}
#endif //VISIONUTILS_PCL_UTILS_COORDINATE_HPP