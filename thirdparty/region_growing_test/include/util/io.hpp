#ifndef _IO_H_
#define _IO_H_

#include <iostream>
#include <fstream>
#include "yaml-cpp/yaml.h"

typedef struct {
    int k_search;
    int min_cluster_size;
    int max_cluster_size;
    int number_of_neighbours;
    double distance_of_neighbours;
    double smoothness_threshold_rad;
    double curvature_threshold;
    double residual_threshold;
    bool smooth_mode_flag;
    bool curvature_test_flag;
    bool residual_test_flag;
    std::string output_dir;
    bool is_coloring;
} rg_param;

rg_param get_param(std::string yaml_path){
    rg_param param;

    YAML::Node param_yaml = YAML::LoadFile(yaml_path);

    param.k_search = param_yaml["k_search"].as<int>();
    param.min_cluster_size = param_yaml["min_cluster_size"].as<int>();
    param.max_cluster_size = param_yaml["max_cluster_size"].as<int>();
    param.number_of_neighbours = param_yaml["number_of_neighbours"].as<int>();
    param.distance_of_neighbours = param_yaml["distance_of_neighbours"].as<double>();
    param.smoothness_threshold_rad = param_yaml["smoothness_threshold_rad"].as<double>();
    param.curvature_threshold = param_yaml["curvature_threshold"].as<double>();
    param.residual_threshold = param_yaml["residual_threshold"].as<double>();
    param.smooth_mode_flag = param_yaml["smooth_mode_flag"].as<bool>();
    param.curvature_test_flag = param_yaml["curvature_test_flag"].as<bool>();
    param.residual_test_flag = param_yaml["residual_test_flag"].as<bool>();
    param.output_dir = param_yaml["output_dir"].as<std::string>();
    param.is_coloring = param_yaml["is_coloring"].as<bool>();
    std::cout << "Finish reading: "<< yaml_path << std::endl;

    return param;
}

void copy_yml(std::string sourceFilename, std::string destFilename)
{
    try {
        std::ifstream sourceFile(sourceFilename, std::ios::binary);
        if (!sourceFile) throw std::runtime_error("入力ファイルを開けませんでした");
        std::ofstream destFile(destFilename, std::ios::binary);
        if (!destFile) throw std::runtime_error("出力ファイルを開けませんでした");
        destFile << sourceFile.rdbuf();
        sourceFile.close();
        destFile.close();
        std::cout << "ファイルが正常にコピーされました。\n";
    } catch (const std::exception& e) {
        std::cerr << "エラー: " << e.what() << std::endl;
        return;
    }
    return;
}

#endif