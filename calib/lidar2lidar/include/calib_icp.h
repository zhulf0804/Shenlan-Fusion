#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/registration/transformation_estimation_svd.h>

#include <Eigen/Core>

#include <gicp/fast_gicp.hpp>
#include <gicp/fast_gicp_st.hpp>

typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

void pcl_icp_match(const PointCloudPtr& src, const PointCloudPtr& base, Eigen::Matrix4d &trans);

void gicp_icp_match(const PointCloudPtr& src, const PointCloudPtr& base, Eigen::Matrix4d &trans);

