#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <filesystem>
#include <regex>
#include <pcl/common/pca.h>
#include <thread>

// #include "yaml-cpp/yaml.h"
#include "../include/rg/region_growing_balado.hpp"
#include "../include/util/io.hpp"

void split_pcd(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, rg_param rg_param)
{
    std::cout << "estimate normals"<< std::endl;
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (rg_param.k_search);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);
    pcl::RegionGrowingBalado<pcl::PointXYZRGB, pcl::Normal> reg;

    reg.setSmoothModeFlag (rg_param.smooth_mode_flag);
    reg.setCurvatureTestFlag (rg_param.curvature_test_flag);
    reg.setResidualTestFlag (rg_param.residual_test_flag);
    reg.setMinClusterSize (rg_param.min_cluster_size); 
    reg.setMaxClusterSize (rg_param.max_cluster_size);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (rg_param.number_of_neighbours); //30
    reg.setDistanceOfNeighbours (rg_param.distance_of_neighbours);
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (rg_param.smoothness_threshold_rad);
    reg.setCurvatureThreshold (rg_param.curvature_threshold);
    reg.setResidualThreshold (rg_param.residual_threshold);

    std::cout << "RegionGrowingBalado" << std::endl;
    std::cout << "Search: Radius(r = "<<rg_param.distance_of_neighbours<<"[m], max_n = "<< rg_param.number_of_neighbours<<")" << std::endl;
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    // TODO
    // std::cout << "Extract unassigned cloud" << std::endl;
    // pcl::PointIndices unassigned;
    // reg.extractUnassigned (unassigned);

    std::cout << "saving output" << std::endl;
    if(rg_param.is_coloring){
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
        std::string output_path = rg_param.output_dir + "/"  + "all.pcd";
        pcl::io::savePCDFileASCII (output_path , *colored_cloud);
    }
    else{   
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        for (size_t i = 0; i < clusters.size(); ++i) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointIndices::Ptr ind(new pcl::PointIndices(clusters[i]));

            extract.setInputCloud (cloud);
            extract.setIndices (ind);
            extract.filter (*extracted_cloud);
            extract.setNegative(false);

            std::string output_path = rg_param.output_dir + "/"  + std::to_string(i) + ".pcd";
            pcl::io::savePCDFileASCII(output_path, *extracted_cloud);
        }

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // pcl::PointIndices::Ptr ind(new pcl::PointIndices(unassigned));
        // extract.setInputCloud (cloud);
        // extract.setIndices (ind);
        // extract.filter (*extracted_cloud);
        // extract.setNegative(false);
        // std::string output_path = rg_param.output_dir + "/"  + "unassigned.pcd";
        // pcl::io::savePCDFileASCII(output_path, *extracted_cloud);
    }
}

int main(int argc, char** argv)
{
    if (argc == 1){
        std::cout <<"no input_path : "<<std::endl;
        return 0;
    }
    else if (argc == 2){
        std::cout <<"no param_ path: "<<std::endl;
        return 0;
    }
    
    // load input file
    std::string input_path = argv[1];
    std::cout <<"input_path: " << input_path << std::endl;
    
    // load yamls
    std::vector<rg_param> rg_params;
    for(int i = 2; i < argc; i++){
        std::cout <<"yaml_path ("<<(i-2)<<") : " << argv[i]<< std::endl;
        std::string yaml_path = argv[i];
        rg_param rg_param = get_param(yaml_path);

        std::filesystem::create_directories(rg_param.output_dir);
        copy_yml(yaml_path, rg_param.output_dir + "/"  + "param.yml");

        rg_params.push_back(rg_param);
    }
    
    // create cloud
    std::cout << "Start reading input file" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::string::size_type idx = input_path.find_last_of('.');
    if(idx != std::string::npos) {
        std::string ext = input_path.substr(idx+1);
        if(ext == "ply"){
            pcl::PLYReader Reader;
            if (Reader.read(input_path, *cloud) == -1)
                {
                    std::cerr << "Failed to read input file: " << input_path << std::endl;
                    return (-1);
                }
            std::cout << "Finish reading input file (.ply)" << std::endl;
        }
        else if(ext == "pcd"){
            if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> (input_path, *cloud) == -1)
                {
                    std::cout << "Cloud reading failed." << std::endl;
                    return (-1);
                }
            std::cout << "Finish reading input file (.pcd)" << std::endl;
        }
        else{
            std::cout << "Cloud reading failed." << std::endl;
            return (-1);
        }
    } else {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    // スレッドを作成して処理を開始
    std::vector<std::thread> threads;
    
    for(auto& rg_param: rg_params) {
        threads.emplace_back(split_pcd, cloud,rg_param);
    }

    // すべてのスレッドの終了を待機
    for(auto& thread : threads) {
        thread.join();
    }
  return 0;
}


