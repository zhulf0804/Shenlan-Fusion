// lidar 2 lidar calibration
#include <chrono>
#include <iostream>
#include <random>
#include <string>
#include <algorithm>

#include <Eigen/Core>

#include "lidar2lidar.h"


void lidar2lidarCalibration(const std::string& path1, const std::string& path2)
{
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_pts(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_pts(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::io::loadPCDFile(path1, *source_pts);
  pcl::io::loadPCDFile(path2, *target_pts);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZINormal>(*source_pts, *source_pts, indices);
  pcl::removeNaNFromPointCloud<pcl::PointXYZINormal>(*target_pts, *target_pts, indices);

  pcl::VoxelGrid<pcl::PointXYZINormal> filter;
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  filter.setInputCloud(source_pts);
  filter.filter(*source_pts);
  filter.setInputCloud(target_pts);
  filter.filter(*target_pts);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;

  sor.setInputCloud(source_pts);
  sor.setMeanK(10);
  sor.setStddevMulThresh(1.0);
  sor.filter(*source_pts);

  sor.setInputCloud(target_pts);
  sor.filter(*target_pts);

  std::cout<<"load pcd"<<std::endl;

  Eigen::Matrix4d tran_mat_icp;
  Eigen::Matrix4d tran_mat_teaser;

  tran_mat_icp.setIdentity();

  // TODO: change to gicp icp match

  // pcl_icp_match(source_pts, target_pts, tran_mat_icp);
  gicp_icp_match(source_pts, target_pts, tran_mat_icp);

  std::cout<<"the calibration result is "<<std::endl<<tran_mat_icp<<std::endl;

  PointCloudPtr transed_cloud(new PointCloud);
  pcl::transformPointCloud(*source_pts, *transed_cloud, tran_mat_icp);

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("trans_viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> origin_color_handle(source_pts, 0, 0, 255);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> pred_color_handle(target_pts, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> target_color_handle(transed_cloud, 0, 255, 0);

  viewer->addPointCloud(source_pts, origin_color_handle, "origin");
  viewer->addPointCloud(target_pts, pred_color_handle, "pred");
  viewer->addPointCloud(transed_cloud, target_color_handle, "target");

  viewer->spin();
  //https://github.com/PointCloudLibrary/pcl/issues/172
  //viewer windows can't close
  viewer->close();
}


int main(int argc, char ** argv){
  std::string path1 = argv[1];
  std::string path2 = argv[2];

  lidar2lidarCalibration(path1, path2);
  return 0;
}