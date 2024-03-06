#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <chrono>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/statistical_outlier_removal.h>

// get current time
std::string getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    return std::ctime(&time);
}

int main()
{
    // show the current time
    std::cout << "程序开始运行时间：" << getCurrentTime() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile("3after.pcd", *cloud) == -1)
    {
        PCL_ERROR("Could not read pcd file!\n");
        return -1;
    }

    // normal estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(50);
    n.compute(*normals);

        // connect normals and coordinates
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    // poisson reconstruction
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    pcl::Poisson<pcl::PointNormal> pn;
    pn.setSearchMethod(tree2);
    pn.setInputCloud(cloud_with_normals);
    pn.setThreads(12);
    pn.setDepth(9);
    pn.setMinDepth(2);
    pn.setScale(1);
    pn.setSolverDivide(3);
    pn.setIsoDivide(8);
    pn.setSamplesPerNode(2);
    pn.setConfidence(false);
    pn.setManifold(false);
    pn.setOutputPolygons(false);

    pcl::PolygonMesh mesh;
    pn.performReconstruction(mesh);

    // visual result
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    viewer->setWindowName(u8"泊松曲面重建");
    viewer->addPolygonMesh(mesh, "my");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "my");
    viewer->initCameraParameters();
    std::cout << "中间环节结束运行时间：" << getCurrentTime() << std::endl;

    // caculate triangle area
    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);

    std::vector<pcl::Vertices> polygons = mesh.polygons;
    std::vector<bool> keep_triangle(polygons.size(), true);  //sign

    for (size_t i = 0; i < polygons.size(); ++i)
    {
        pcl::Vertices polygon = polygons[i];

        // get triangle vertices
        pcl::PointXYZ pt1 = mesh_cloud->points[polygon.vertices[0]];
        pcl::PointXYZ pt2 = mesh_cloud->points[polygon.vertices[1]];
        pcl::PointXYZ pt3 = mesh_cloud->points[polygon.vertices[2]];

        // caculate normal 1和nornal 2
        Eigen::Vector3f vec1(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z);
        Eigen::Vector3f vec2(pt3.x - pt1.x, pt3.y - pt1.y, pt3.z - pt1.z);

        // caculate accumulation
        Eigen::Vector3f cross_product = vec1.cross(vec2);

        // caculate triangle area
        double area = 0.5 * cross_product.norm();

        // decide whether to retain
        double area_threshold = 0.0003;  // set threshold
        if (area > area_threshold)
        {
            keep_triangle[i] = false;
        }
    }

    // extraction triangle
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr filtered_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    std::vector<pcl::Vertices> filtered_polygons;

    extract.setInputCloud(mesh_cloud);
    for (size_t i = 0; i < polygons.size(); ++i)
    {
        if (keep_triangle[i])
        {
            filtered_polygons.push_back(polygons[i]);
        }
    }

    // creat a smart pointer
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    for (const auto& polygon : filtered_polygons)
    {
        indices->indices.insert(indices->indices.end(), polygon.vertices.begin(), polygon.vertices.end());
    }

    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*filtered_cloud);

    // construct PolygonMesh
    pcl::PolygonMesh filtered_mesh;
    filtered_mesh.polygons = filtered_polygons;
    filtered_mesh.cloud = mesh.cloud;

    pcl::io::savePLYFile("3after_ok.ply", filtered_mesh);

    std::cout << "程序结束运行时间：" << getCurrentTime() << std::endl;

    // visual result
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_filtered(new pcl::visualization::PCLVisualizer("Filtered 3D viewer"));
    viewer_filtered->setBackgroundColor(1, 1, 1);
    viewer_filtered->setWindowName(u8"去除平滑部分的结果");
    viewer_filtered->addPolygonMesh(filtered_mesh, "filtered_mesh");
    viewer_filtered->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "filtered_mesh");
    viewer_filtered->initCameraParameters();
    while (!viewer_filtered->wasStopped())
    {
        viewer_filtered->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 0;
}