//
// Created by Mengfanyong on 2025/12/12.
//

#ifndef VISIONUTILS_FILTERS_HPP
#define VISIONUTILS_FILTERS_HPP
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/sac_model.h>
namespace pcl_utils::filters {




    template<typename PointT>
    void projectInlier(
        const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
        pcl::SacModel sac_model,
        const pcl::ModelCoefficients::Ptr  &coefficients,
        const typename pcl::PointCloud<PointT>::Ptr &projected_cloud
        )
    {
        // Now project the points onto the detected plane
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(sac_model);
        proj.setInputCloud(cloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*projected_cloud);

    }







    /**
     * @brief 使用AABB盒子裁剪点云
     *
     * @tparam PointT 点云类型
     * @param cloud 输入点云
     * @param min_pt AABB最小点
     * @param max_pt AABB最大点
     * @param cropped_cloud 输出裁剪后的点云
     */
    template<typename PointT>
    void cropPointCloudWithAABB(
        const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
        const Eigen::Vector4f &min_pt,
        const Eigen::Vector4f &max_pt,
        const typename pcl::PointCloud<PointT>::Ptr &cropped_cloud
    ) {
        pcl::CropBox<PointT> crop_box;
        crop_box.setMin(min_pt);
        crop_box.setMax(max_pt);
        crop_box.setInputCloud(cloud);
        crop_box.filter(*cropped_cloud);
    }


    /*!
     *
     * @brief 使用体素网格法对点云进行下采样
     *
     * @tparam PointT 点云类型
     * @param cloud 输入点云
     * @param leaf_size 体素大小
     * @param downsampled_cloud 输出下采样后的点云
     */
    template<typename PointT>
    void voxelGridDownsample(
        const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
        float leaf_size,
        const typename pcl::PointCloud<PointT>::Ptr &downsampled_cloud
    ) {
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setInputCloud(cloud);
        voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid.filter(*downsampled_cloud);
    }

} // namespace pcl_utils::filters



#endif //VISIONUTILS_FILTERS_HPP