#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <vector>
#include <cmath>

void voxelizePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, 
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree,
    float resolution)
{
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
}

void computeVoxelFeatures(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, 
    std::vector<pcl::Normal> &normals, 
    std::vector<float> &residuals)
{
    std::vector<int> point_indices;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;

    // Octreeの各ボクセルごとに特徴を計算
    for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it)
    {
        pcl::PointXYZ centroid;
        pcl::computeCentroid(*(it.getLeafContainer().getPointIndicesVector()), centroid);

        pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_points(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto idx : it.getLeafContainer().getPointIndicesVector())
        {
            voxel_points->points.push_back(octree.getInputCloud()->points[idx]);
        }

        // 法線ベクトルを計算
        pcl::Normal normal;
        normal_estimator.setInputCloud(voxel_points);
        pcl::PointCloud<pcl::Normal>::Ptr normals_voxel(new pcl::PointCloud<pcl::Normal>);
        normal_estimator.compute(*normals_voxel);
        normal = normals_voxel->points[0];
        normals.push_back(normal);

        // 残差値の計算
        float residual = 0.0f;
        for (const auto &point : voxel_points->points)
        {
            float distance = std::abs((normal.normal_x * point.x) +
                                      (normal.normal_y * point.y) +
                                      (normal.normal_z * point.z));
            residual += distance * distance;
        }
        residuals.push_back(std::sqrt(residual / voxel_points->points.size()));
    }
}

void octreeRegionGrowingPhaseA(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, 
    std::vector<int> &segment_ids, 
    const std::vector<pcl::Normal> &normals, 
    const std::vector<float> &residuals,
    float normal_threshold, 
    float residual_threshold)
{
    int segment_id = 0;
    std::vector<bool> processed(octree.getLeafCount(), false);

    for (auto it = octree.leaf_begin(); it != octree.leaf_end(); ++it)
    {
        int leaf_idx = std::distance(octree.leaf_begin(), it);
        if (processed[leaf_idx])
            continue;

        // 新しいセグメントの開始
        std::vector<int> to_grow;
        to_grow.push_back(leaf_idx);
        processed[leaf_idx] = true;

        while (!to_grow.empty())
        {
            int current_leaf = to_grow.back();
            to_grow.pop_back();

            // 隣接するボクセルの探索
            std::vector<int> neighbors;
            octree.getApproxIntersectedVoxelIndices(current_leaf, neighbors);

            for (int neighbor : neighbors)
            {
                if (processed[neighbor])
                    continue;

                // 法線ベクトルの角度と残差値をチェック
                float angle_diff = pcl::getAngle3D(normals[current_leaf], normals[neighbor]);
                float residual_diff = std::abs(residuals[current_leaf] - residuals[neighbor]);

                if (angle_diff < normal_threshold && residual_diff < residual_threshold)
                {
                    to_grow.push_back(neighbor);
                    processed[neighbor] = true;
                }
            }
        }

        segment_ids[leaf_idx] = segment_id++;
    }
}

void refineSegmentsPhaseB(
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> &octree, 
    std::vector<int> &segment_ids, 
    const std::vector<int> &boundary_voxels,
    const std::vector<pcl::Normal> &normals, 
    float normal_threshold)
{
    for (int boundary_voxel : boundary_voxels)
    {
        std::vector<int> neighbors;
        octree.getApproxIntersectedVoxelIndices(boundary_voxel, neighbors);

        for (int neighbor : neighbors)
        {
            if (segment_ids[neighbor] == -1) // 未割り当て
            {
                float angle_diff = pcl::getAngle3D(normals[boundary_voxel], normals[neighbor]);

                if (angle_diff < normal_threshold)
                {
                    segment_ids[neighbor] = segment_ids[boundary_voxel];
                }
            }
        }
    }
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/aichi2204/cloud2mesh_tool/src/c++/supervoxel/milk_cartoon_all_small_clorox.pcd", *cloud);

    float resolution = 0.05f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    voxelizePointCloud(cloud, octree, resolution);

    std::vector<pcl::Normal> normals;
    std::vector<float> residuals;
    computeVoxelFeatures(octree, normals, residuals);

    std::vector<int> segment_ids(octree.getLeafCount(), -1);
    float normal_threshold = 10.0f * (M_PI / 180.0f);
    float residual_threshold = 0.05f;

    octreeRegionGrowingPhaseA(octree, segment_ids, normals, residuals, normal_threshold, residual_threshold);

    // フェーズB: 境界の精錬
    std::vector<int> boundary_voxels = getBoundaryVoxels(segment_ids);
    refineSegmentsPhaseB(octree, segment_ids, boundary_voxels, normals, normal_threshold);

    // 結果の可視化
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

    // 各セグメントを色分けして表示
    for (int i = 0; i < segment_ids.size(); ++i)
    {
        if (segment_ids[i] != -1)
        {
            std::stringstream ss;
            ss << "segment_" << segment_ids[i];
            pcl::PointCloud<pcl::PointXYZ>::Ptr segment_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            getPointsInSegment(octree, i, *segment_cloud);
            viewer->addPointCloud<pcl::PointXYZ>(segment_cloud, ss.str());
        }
    }

    viewer->spin();

    return 0;
}