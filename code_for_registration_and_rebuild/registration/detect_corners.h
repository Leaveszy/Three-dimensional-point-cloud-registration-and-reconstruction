#ifndef DETECT_CORNERS_H
#define DETECT_CORNERS_H

#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/kdtree.h>

void detectCorners(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& corner_points, float curvature_threshold = 0.08, float angle_threshold = 90.0);

#endif // DETECT_CORNERS_H
