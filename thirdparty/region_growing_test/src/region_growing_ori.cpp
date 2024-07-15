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

int
main ()
{
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGBL> ("/home/aichi2204/Downloads/region_growing_tutorial.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  // pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
  // pcl::PLYReader Reader;
  // if (Reader.read("/home/aichi2204/Documents/bkl2go/20240412-library1/aichi-20240412-library1.ply", *cloud) == -1)
  // {
  //   std::cout << "Cloud reading failed." << std::endl;
  //   return (-1);
  // }

  // smoothness:true
  // curvature:true
  // residual:false
  pcl::search::Search<pcl::PointXYZRGBL>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBL>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGBL, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud(*cloud, *indices);

  pcl::RegionGrowing<pcl::PointXYZRGBL, pcl::Normal> reg;
  reg.setSmoothModeFlag (true);
  reg.setCurvatureTestFlag (true);
  reg.setResidualTestFlag (true);

  reg.setMinClusterSize (10);//(50);
  reg.setMaxClusterSize (10000000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (50);
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (0.5);
  reg.setCurvatureThreshold (0.05);
  reg.setResidualThreshold (0.5);

  /*
  <ni o nj < 3.0 and r > 1/1.0(c<1.0)
  */
  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << std::endl;
  std::cout << "These are the indices of the points of the initial" <<
  std::endl << "cloud that belong to the first cluster:" << std::endl;
  std::size_t counter = 0;
  // while (counter < clusters[0].indices.size ())
  // {
  //   std::cout << clusters[0].indices[counter] << ", ";
  //   counter++;
  //   if (counter % 10 == 0)
  //     std::cout << std::endl;
  // }
  // std::cout << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::io::savePCDFileASCII ("../data/pcd_data2.pcd" , *colored_cloud);

  return (0);
}