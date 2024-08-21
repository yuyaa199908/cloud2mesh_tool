#include <iostream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

int main(void) {
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZRGBL> ("/home/aichi2204/Documents/bkl2go/20240412-library1/aichi-20240412-library1.ply", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PLYReader Reader;
  // if (Reader.read("/home/aichi2204/Documents/bkl2go/aichi-20240412-library1.ply", *cloud) == -1)
  // {
  //   std::cout << "Cloud reading failed." << std::endl;
  //   return (-1);
  // }

  std::cout << (*cloud)[10] << std::endl;

  return (0);
}