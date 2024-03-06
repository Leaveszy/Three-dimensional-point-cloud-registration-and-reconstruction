#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
    // Read the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("8.pcd", *cloud);

    // Create StatisticalOutlierRemoval object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    // Set the input point cloud
    sor.setInputCloud(cloud);

    // Set filter parameters
    sor.setMeanK(100);         // Sets the number of neighborhood points used to calculate the average distance
    sor.setStddevMulThresh(10);  // Set the standard deviation multiplier threshold

    // Application filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*filtered_cloud);

    // Save the filtered point cloud to the file
    pcl::io::savePCDFileBinary("8after.pcd", *filtered_cloud);

    // Visual result
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    viewer.setBackgroundColor(0, 0, 0);
   // viewer.addPointCloud(cloud, "original_cloud");
   // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");

    viewer.addPointCloud(filtered_cloud, "filtered_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "filtered_cloud");  

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}



