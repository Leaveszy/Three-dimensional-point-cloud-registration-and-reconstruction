#include "detect_corners.h"

float calculateAngle(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2, const pcl::PointXYZRGB& p3)
{
    pcl::PointXYZRGB v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
    pcl::PointXYZRGB v2(p3.x - p2.x, p3.y - p2.y, p3.z - p2.z);

    float dot_product = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    float v1_length = std::sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
    float v2_length = std::sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
    return std::acos(dot_product / (v1_length * v2_length)) * 180.0 / M_PI;
}

void detectCorners(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& corner_points, float curvature_threshold, float angle_threshold)
{
    // Compute normals and curvature
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(50);
    ne.compute(*normals);

    int m_nearest_neighbors = 8;

    // Extract corner points
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (normals->points[i].curvature > curvature_threshold) {
            pcl::PointXYZRGB current_point = cloud->points[i];
            std::vector<int> m_indices;
            std::vector<float> m_sqr_distances;
            tree->nearestKSearch(current_point, m_nearest_neighbors, m_indices, m_sqr_distances);

            bool is_corner = true;
            for (size_t j = 1; j < m_indices.size() - 1; ++j) {
                float angle = calculateAngle(cloud->points[m_indices[j - 1]], cloud->points[m_indices[j]], cloud->points[m_indices[j + 1]]);
                if (angle < angle_threshold) {
                    is_corner = false;
                    break;
                }
            }

            if (is_corner) {
                corner_points->points.push_back(current_point);
            }
        }
    }
    //Lalala modify 
   // corner_points->width = corner_points->points.size();
   // corner_points->height = 1;
}
