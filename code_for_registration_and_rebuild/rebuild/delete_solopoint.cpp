#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>
#include <boost/thread.hpp>
#include <omp.h>
#include <chrono>

// get current time
std::string getCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    return std::ctime(&time);
}

void processPolygons(const std::vector<pcl::Vertices>& polygons,
    std::vector<pcl::Vertices>& filtered_polygons,
    std::vector<std::vector<int>>& adjacency_list,
    int start_idx,
    int end_idx,
    boost::mutex& mtx)
{
    for (int i = start_idx; i < end_idx; ++i)
    {
        const auto& polygon = polygons[i];
        const auto& vertex_indices = polygon.vertices;

        // count 
        int adjacent_polygons = 0;
        for (int j = 0; j < vertex_indices.size(); ++j)
        {
            adjacent_polygons += adjacency_list[vertex_indices[j]].size();
        }

        // decide
        if (adjacent_polygons >14)
        {
            boost::lock_guard<boost::mutex> lock(mtx);
            filtered_polygons.push_back(polygon);
        }
    }
}

int main(int argc, char** argv)
{

    // show the start time
    std::cout << "程序开始运行时间：" << getCurrentTime() << std::endl;

    // PLY
    pcl::PolygonMesh mesh;
    pcl::PolygonMesh mesh2;
    pcl::io::loadPLYFile("3after_ok.ply", mesh);
    pcl::io::loadPLYFile("3after_ok.ply", mesh2);

    // get the infirmation
    std::vector<pcl::Vertices> polygons = mesh.polygons;

    // builds list of vertices
    std::vector<std::vector<int>> adjacency_list(mesh.cloud.width * mesh.cloud.height);

    for (int i = 0; i < polygons.size(); ++i)
    {
        const auto& polygon = polygons[i];
        const auto& vertex_indices = polygon.vertices;

        for (int j = 0; j < vertex_indices.size(); ++j)
        {
            adjacency_list[vertex_indices[j]].push_back(i);
        }
    }

    // get hardware parallelism
    int num_threads = 0;
#pragma omp parallel
    {
        num_threads = omp_get_num_threads();
    }

    // create a shared mutex
    boost::mutex mtx;

    // create the number of thread groups
    std::vector<boost::thread> threads;

    // per-thread processing
    int polygons_per_thread = polygons.size() / num_threads;

    // the updated surface
    std::vector<pcl::Vertices> filtered_polygons;

    // Start thread
#pragma omp parallel for
    for (int i = 0; i < num_threads; ++i)
    {
        int start_idx = i * polygons_per_thread;
        int end_idx = (i == num_threads - 1) ? polygons.size() : (i + 1) * polygons_per_thread;

        // Create a thread and pass handlers and arguments to the thread
        threads.emplace_back(processPolygons, std::ref(polygons), std::ref(filtered_polygons), std::ref(adjacency_list), start_idx, end_idx, std::ref(mtx));
    }

    // Wait for all threads to complete
    for (auto& thread : threads)
    {
        thread.join();
    }

    // Set new polygon information
    mesh.polygons = filtered_polygons;

    // Save the processed Mesh to the PLY file
    pcl::io::savePLYFile("3_ok.ply", mesh);

    std::cout << "程序结束运行时间：" << getCurrentTime() << std::endl;

    // Create a visual window
    pcl::visualization::PCLVisualizer viewer("after");
    pcl::visualization::PCLVisualizer viewer2("before");

    // Add a polygonal mesh to the visualization window
    viewer.addPolygonMesh(mesh, "mesh");
    viewer2.addPolygonMesh(mesh2, "mesh2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "mesh");
    viewer2.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "mesh2");
    // Set the visual window background color
    viewer.setBackgroundColor(1, 1, 1);
    viewer2.setBackgroundColor(1, 1, 1);

    // Wait for the user to close the window
    viewer.spin();
    viewer2.spin();

    return 0;
}























