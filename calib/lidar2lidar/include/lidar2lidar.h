#pragma once

#include "calib_icp.h"

typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

void lidar2lidarCalibration(const std::string& path1, const std::string& path2);