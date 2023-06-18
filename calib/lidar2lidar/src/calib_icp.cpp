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

  norm_est.setNumberOfThreads(8);
  norm_est.setSearchMethod(kdtree);
  norm_est.setKSearch(20);

  norm_est.setInputCloud(src);
  norm_est.compute(*src);
  norm_est.setInputCloud(base);
  norm_est.compute(*base);

  pcl::IterativeClosestPointWithNormals<PointT, PointT, float>::Ptr icp(new pcl::IterativeClosestPointWithNormals<PointT, PointT>);

  // 设置源点云和目标点云
  icp->setInputSource(src);
  icp->setInputTarget(base);

  icp->setMaxCorrespondenceDistance(5.0);

  // 设置最大迭代次数
  icp->setMaximumIterations(100);

  // 设置收敛判断阈值
  icp->setTransformationEpsilon(1e-4);

  // 设置配准的距离阈值
  icp->setEuclideanFitnessEpsilon(1e-3);

  Eigen::Matrix4f init_pose = trans.cast<float>();
  pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>);
  icp->align(*aligned, init_pose);

  std::cout<<"align icp final"<<std::endl;
  if (icp->hasConverged()) {
    trans = icp->getFinalTransformation().cast<double>();
  } else {
    std::cout << "align failed" << std::endl;
  }

  ////////////////////////////////////////////////////////////

}

void gicp_icp_match(const PointCloudPtr& src, const PointCloudPtr& base, Eigen::Matrix4d &trans) {

  // TODO homework

  /////////////////////////////////////////////////////////////////////////////
  //// reference: https://github.com/SMRT-AIST/fast_gicp

  fast_gicp::FastGICPSingleThread<pcl::PointXYZ, pcl::PointXYZ> fgicp_st;

  pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
  for(int i = 0; i < src->points.size(); i++){
    source->points.emplace_back(src->points[i].x, src->points[i].y, src->points[i].z);
  }
  for(int i = 0; i < base->points.size(); i++){
    target->points.emplace_back(base->points[i].x, base->points[i].y, base->points[i].z);
  }


  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);

  double fitness_score = 0.0;

  // single run
  // fast_gicp reuses calculated covariances if an input cloud is the same as the previous one
  // to prevent this for benchmarking, force clear source and target clouds
  fgicp_st.clearTarget();
  fgicp_st.clearSource();
  fgicp_st.setInputTarget(target);
  fgicp_st.setInputSource(source);
  fgicp_st.align(*aligned);
  fitness_score = fgicp_st.getFitnessScore();
  std::cout << "fitness score: " << fitness_score << std::endl;

  trans = fgicp_st.getFinalTransformation().cast<double>();

  /////////////////////////////////////////////////////////////////////////////////

}
