#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " point_cloud1.pcd point_cloud2.pcd" << std::endl;
        return -1;
    }

    // 1つ目の点群を読み込む
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud1) == -1) {
        std::cerr << "Couldn't read file " << argv[1] << std::endl;
        return -1;
    }

    // 2つ目の点群を読み込む
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud2) == -1) {
        std::cerr << "Couldn't read file " << argv[2] << std::endl;
        return -1;
    }

    std::cout << "read 2 files " <<  std::endl;

    // KdTreeを構築する
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);
    std::cout << "create kdtree" <<  std::endl;

    // 最小距離を初期化
    float min_distance = std::numeric_limits<float>::infinity();

    // 1つ目の点群の各点について、最近傍点を見つけ、最小距離を計算する
    for (const auto& point : *cloud1) {
        std::vector<int> indices(1);
        std::vector<float> distances(1);
        std::cout << point <<  std::endl;
        kdtree.nearestKSearch(point, 1, indices, distances);
        if (distances[0] < min_distance) {
            min_distance = distances[0];
        }
    }

    std::cout << "Minimum distance between the point clouds: " << min_distance << std::endl;

    return 0;
}