#include "calib_icp.h"

void pcl_icp_match(const PointCloudPtr& src, const PointCloudPtr& base, Eigen::Matrix4d &trans) {

  // TODO homework

  /////////////////////////////////////////////////////////////////////////
  //// feel free to refer to 
  //// https://pointclouds.org/documentation/classpcl_1_1_normal_estimation_o_m_p.html 
  //// https://pointclouds.org/documentation/classpcl_1_1search_1_1_kd_tree.html
  //// https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point_with_normals.html
  //// https://pointclouds.org/documentation/classpcl_1_1_registration.html
  //// or something else for this homework

  pcl::NormalEstimationOMP<PointT, PointT> norm_est;
  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());

















  std::cout<<"align icp final"<<std::endl;

  // if (icp.hasConverged()) {
  //   trans = icp.getFinalTransformation().cast<double>();
  // } else {
  //   std::cout << "align failed" << std::endl;
  // }

  ////////////////////////////////////////////////////////////

}

void gicp_icp_match(const PointCloudPtr& src, const PointCloudPtr& base, Eigen::Matrix4d &trans) {

  // TODO homework

  /////////////////////////////////////////////////////////////////////////////
  //// reference: https://github.com/SMRT-AIST/fast_gicp

  fast_gicp::FastGICPSingleThread<pcl::PointXYZ, pcl::PointXYZ> fgicp_st;

















  trans = fgicp_st.getFinalTransformation().cast<double>();

  /////////////////////////////////////////////////////////////////////////////////

}
