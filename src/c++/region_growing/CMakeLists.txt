cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcd_write)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# https://qiita.com/moch517/items/132b481564909687c646
# set(CMAKE_CXX_STANDARD 11)
include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.7.0 # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
  message(STATUS "Fetching yaml-cpp...")
  FetchContent_Populate(yaml-cpp)
  add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()

add_executable (segment_balado src/segment_balado.cpp include/util/io.hpp include/rg/region_growing_balado.hpp)
target_link_libraries (segment_balado ${PCL_LIBRARIES})
target_link_libraries (segment_balado PUBLIC yaml-cpp::yaml-cpp)

add_executable (segment_poux src/segment_poux.cpp include/util/io.hpp include/rg/region_growing_poux.hpp)
target_link_libraries (segment_poux ${PCL_LIBRARIES})
target_link_libraries (segment_poux PUBLIC yaml-cpp::yaml-cpp)

add_executable (region_growing_ori src/region_growing_ori.cpp)
target_link_libraries (region_growing_ori ${PCL_LIBRARIES})
