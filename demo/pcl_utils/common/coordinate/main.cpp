//
// Created by mfy on 2025/12/30.
//

#include <io/pcl_utils_io_txt.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <limits>
#include <mutex>
#include <pcl/common/transforms.h>

#include "pcl_utils/visualization/pcl_utils_vis_events.hpp"
#include "visualization/pcl_utils_visualization.hpp"
using namespace pcl_utils;

#include <Eigen/Dense>
#include <iostream>
#include <stdexcept>
#include "pcl_utils/visualization/pcl_utils_visualization.hpp"
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

int main() {
    std::string file_1 = std::string(DEMO_PATH) + "/data/1.txt";
    std::string file_2 = std::string(DEMO_PATH) + "/data/2.pcd";
    auto cloud_1 = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    auto cloud_2 = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

    if (pcl::io::loadPCDFile(file_2, *cloud_2) == 0) {

        // 当前点云的上轴是 +, 前轴是 Z+
        Eigen::Vector3f up_current(0,1,0);
        Eigen::Vector3f forward_current(0,0,1);

        Eigen::Matrix3f R = computeTransform(up_current, forward_current);

        std::cout << "Rotation matrix from current to target:\n" << R << std::endl;

        // 测试点变换
        Eigen::Vector3f p_current(1,2,3);
        Eigen::Vector3f p_target = R * p_current;

        std::cout << "Point in current coords: " << p_current.transpose() << std::endl;
        std::cout << "Point in target coords: " << p_target.transpose() << std::endl;
        auto after_trans_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        Eigen::Matrix4f  transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = R;
        pcl::transformPointCloud(*cloud_2, *after_trans_cloud, transform);
        pcl_utils::visualization::showPointCloud<pcl::PointXYZ>(cloud_2,after_trans_cloud,"cloud_2,after_trans_cloud",1,2,1);

    }

    if (pcl_utils::io::readTXTToPCLXYZ(file_1, cloud_1)) {

        // 当前点云的上轴是 +, 前轴是 Z+
        Eigen::Vector3f up_current(0,0,1);
        Eigen::Vector3f forward_current(0,-1,0);

        Eigen::Matrix3f R = computeTransform(up_current, forward_current);

        std::cout << "Rotation matrix from current to target:\n" << R << std::endl;

        // 测试点变换
        Eigen::Vector3f p_current(1,2,3);
        Eigen::Vector3f p_target = R * p_current;

        std::cout << "Point in current coords: " << p_current.transpose() << std::endl;
        std::cout << "Point in target coords: " << p_target.transpose() << std::endl;
        auto after_trans_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        Eigen::Matrix4f  transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = R;
        pcl::transformPointCloud(*cloud_1, *after_trans_cloud, transform);
        pcl_utils::visualization::showPointCloud<pcl::PointXYZ>(cloud_1,after_trans_cloud,"cloud_1,after_trans_cloud");
    }

    return 0;
}
