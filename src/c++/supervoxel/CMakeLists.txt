cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(supervoxel_clustering)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (supervoxel_clustering src/supervoxel_clustering.cpp)
target_link_libraries (supervoxel_clustering ${PCL_LIBRARIES})

add_executable (octree_based src/octree_based.cpp)
target_link_libraries (octree_based ${PCL_LIBRARIES})