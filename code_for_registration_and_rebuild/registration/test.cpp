#include <iostream>
#include "detect_corners.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <chrono>
#include <pcl/octree/octree_search.h>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::visualization;



// Perform FPFH-SAC-IA registration at a given resolution level
Eigen::Matrix4f registerAtResolution(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud,
    const float voxel_resolution, pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>& sac_ia)
{
    // Downsample the source and target clouds using voxel grid filtering
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_source(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_target(new pcl::PointCloud<pcl::PointXYZRGB>);

    voxel_grid.setInputCloud(source_cloud);
    voxel_grid.filter(*downsampled_source);

    voxel_grid.setInputCloud(target_cloud);
    voxel_grid.filter(*downsampled_target);

    // Compute surface normals for the downsampled source and target clouds
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr  target_normals(new pcl::PointCloud<pcl::Normal>);

    normal_estimation.setInputCloud(downsampled_source);
    normal_estimation.setRadiusSearch(0.5);
    normal_estimation.compute(*source_normals);

    normal_estimation.setInputCloud(downsampled_target);
    normal_estimation.compute(*target_normals);

    // Compute FPFH features for the downsampled source and target clouds
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);

    fpfh_estimation.setInputCloud(downsampled_source);
    fpfh_estimation.setInputNormals(source_normals);
    fpfh_estimation.setRadiusSearch(0.6);
    fpfh_estimation.compute(*source_features);

    fpfh_estimation.setInputCloud(downsampled_target);
    fpfh_estimation.setInputNormals(target_normals);
    fpfh_estimation.compute(*target_features);

    // Perform FPFH-SAC-IA registration
    sac_ia.setInputSource(downsampled_source);
    sac_ia.setInputTarget(downsampled_target);
    sac_ia.setSourceFeatures(source_features);
    sac_ia.setTargetFeatures(target_features);
    sac_ia.setMaximumIterations(50);


    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    sac_ia.align(*aligned_cloud);


    // output the result
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << sac_ia.getFinalTransformation() << std::endl;

    // Returns the registered transformation matrix
    return sac_ia.getFinalTransformation();

}



int main(int argc, char** argv)
{
    auto start = std::chrono::high_resolution_clock::now();

   
    pcl::PointCloud<PointXYZRGB>::Ptr source(new pcl::PointCloud<PointXYZRGB>);
    pcl::PointCloud<PointXYZRGB>::Ptr source_copy(new pcl::PointCloud<PointXYZRGB>);
    pcl::PointCloud<PointXYZRGB>::Ptr target(new pcl::PointCloud<PointXYZRGB>);
    loadPCDFile<pcl::PointXYZRGB>("outdoor1_2noise.pcd", *source);
    loadPCDFile<pcl::PointXYZRGB>("outdoor1_2noise.pcd", *source_copy);
    loadPCDFile<pcl::PointXYZRGB>("outdoor2.pcd", *target);

    float curvature_threshold = 0.03;
    float angle_threshold = 90.0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points1(new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points1_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points2(new pcl::PointCloud<pcl::PointXYZRGB>);
    detectCorners(source, corner_points1, curvature_threshold, angle_threshold);

     curvature_threshold = 0.01;

    detectCorners(target, corner_points2, curvature_threshold, angle_threshold);
    
    
    pcl::io::savePCDFileBinary("outdoor1_2_corner.pcd", *corner_points1);
    pcl::io::savePCDFileBinary("outdoor2_corner.pcd", *corner_points2);

   // corner_points1_copy = corner_points1;
    //cout << "pw:" << corner_points1->width << endl;


    std::cout << "Number of corner points 1: " << corner_points1->points.size() << std::endl;
    std::cout << "Number of corner points 2: " << corner_points2->points.size() << std::endl;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points1_2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points1_2_copy(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points2_2(new pcl::PointCloud<pcl::PointXYZRGB>);

    loadPCDFile<pcl::PointXYZRGB>("outdoor1_2_corner.pcd", *corner_points1_2);
    loadPCDFile<pcl::PointXYZRGB>("outdoor1_2_corner.pcd", *corner_points1_2_copy);
    loadPCDFile<pcl::PointXYZRGB>("outdoor2_corner.pcd", *corner_points2_2);

    // Set resolution parameters
    float voxel_resolution = 0.04; 
    float voxel_resolution_step = 0.01; 
    float max_voxel_resolution = 0.02; 

    // build sac_ia
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia;

    // define the final transformation
    Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity();

    while (voxel_resolution >= max_voxel_resolution)
    {
        // start point cloud registration
        Eigen::Matrix4f transformation = registerAtResolution(corner_points1_2, corner_points2_2, voxel_resolution, sac_ia);

        // cumulative transformation matrix
        final_transformation = transformation * final_transformation;

        // output the result
        std::cout << "Voxel Resolution: " << voxel_resolution << std::endl;
        std::cout << "Transformation Matrix:" << std::endl;
        std::cout << transformation << std::endl;

        // gupdate the reference point cloud
        pcl::PointCloud<PointXYZRGB>::Ptr transformed_source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*corner_points1_2_copy, *transformed_source_cloud, final_transformation);
        *corner_points1_2 = *transformed_source_cloud;

        // Downsample the new source cloud for the next level
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
        voxel_grid.setLeafSize(voxel_resolution, voxel_resolution, voxel_resolution);
        voxel_grid.setInputCloud(corner_points1_2);
        voxel_grid.filter(*corner_points1_2);
        //voxel_grid.setInputCloud(corner_points2_2);
        //voxel_grid.filter(*corner_points2_2);

        // uptade the resolution
        voxel_resolution -= voxel_resolution_step;
    }

    // output the final Transformation Matrix
    std::cout << "Final Transformation Matrix:" << std::endl;
    std::cout << final_transformation << std::endl;


    pcl::PointCloud<PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::transformPointCloud(*source_copy, *aligned_cloud, final_transformation);


    //Accurate registration drop sampling

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_aligned_2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_target_2(new pcl::PointCloud<pcl::PointXYZRGB>);

    float voxel_resolution_2 = 0.01;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_2;
    voxel_grid_2.setLeafSize(voxel_resolution_2, voxel_resolution_2, voxel_resolution_2);

    voxel_grid_2.setInputCloud(target);
    voxel_grid_2.filter(*downsampled_target_2);

    voxel_grid_2.setInputCloud(aligned_cloud);
    voxel_grid_2.filter(*downsampled_aligned_2);



    // Initializes the NDT registration object
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.25);

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree_source(0.01);
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree_target(0.01);
    octree_source.setInputCloud(downsampled_aligned_2);
    octree_source.addPointsFromInputCloud();
    octree_target.setInputCloud(downsampled_target_2);
    octree_target.addPointsFromInputCloud();

    // set the input
    ndt.setInputTarget(downsampled_target_2);

            
            ndt.setInputSource(downsampled_aligned_2);
            
            ndt.setInputTarget(downsampled_target_2);
           
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            ndt.align(*output_cloud);
      
            Eigen::Matrix4f transformation = ndt.getFinalTransformation();
            //std::cout << "Transformation matrix:" << std::endl << transformation << std::endl;
 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*aligned_cloud, *aligned_cloud_2, ndt.getFinalTransformation());
    pcl::io::savePCDFileBinary("7_2_over.pcd", *aligned_cloud_2);


/*



    // Build KD-tree for target cloud
    auto target_kdtree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    target_kdtree->setInputCloud(downsampled_target_2);

    // Build KD-tree for input cloud
    auto input_kdtree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    input_kdtree->setInputCloud(downsampled_aligned_2);


    // Set NDT registration parameters
    pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.25);
    ndt.setResolution(1);
    // Set NDT registration input
    ndt.setInputTarget(downsampled_target_2);
    ndt.setInputSource(downsampled_aligned_2);

    // Set KDTREE for NDT registration
    ndt.setSearchMethodTarget(target_kdtree);
    ndt.setSearchMethodSource(input_kdtree);

    // Perform NDT registration
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
    ndt.align(*aligned_cloud_1);
    pcl::transformPointCloud(*aligned_cloud, *aligned_cloud_2, ndt.getFinalTransformation());
    pcl::io::savePCDFileBinary("7_2_over.pcd", *aligned_cloud_2);

    // Print registration results
    std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
    std::cout << "Fitness score: " << ndt.getFitnessScore() << std::endl;
    std::cout << "Transformation matrix:" << std::endl << ndt.getFinalTransformation() << std::endl;


    */

    //output the time
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    std::cout << "程序运行时间: " << duration << " 微秒" << std::endl;


    // Visualize the original and aligned point clouds
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
    viewer.setBackgroundColor(1.0, 1.0, 1.0);

    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> source_color(source_cloud_copy, 255, 0, 0);

   
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> aligned_color(aligned_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> aligned_2_color(aligned_cloud_2, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> target_color(target, 0, 0, 255);


    //  viewer.addPointCloud(source_cloud_copy, source_color, "source_cloud");

    //viewer.addPointCloud(aligned_cloud, aligned_color, "aligned_cloud");
    viewer.addPointCloud(aligned_cloud_2, aligned_2_color, "aligned_2_cloud");
    viewer.addPointCloud(target, target_color, "target_cloud");

    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");

    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "aligned_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "aligned_2_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");
    viewer.removeCoordinateSystem();
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }



    return 0;
}