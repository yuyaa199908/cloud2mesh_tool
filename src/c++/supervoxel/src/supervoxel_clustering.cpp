#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <filesystem>
//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

// void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
//                                        PointCloudT &adjacent_supervoxel_centers,
//                                        std::string supervoxel_name,
//                                        pcl::visualization::PCLVisualizer::Ptr & viewer);


int
main (int argc, char ** argv)
{
  if (argc < 3)
  {
    pcl::console::print_error ("Syntax is: %s <pcd-file>  <out-dir>\n "
                                "--NT Dsables the single cloud transform \n"
                                "-v <voxel resolution>\n-s <seed resolution>\n"
                                "-c <color weight> \n-z <spatial weight> \n"
                                "-n <normal_weight>\n", argv[0]);
    return (1);
  }
  

  PointCloudT::Ptr cloud (new PointCloudT);
  pcl::console::print_highlight ("Loading point cloud...\n");
  if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud))
  {
    pcl::console::print_error ("Error loading cloud file!\n");
    return (1);
  }

  // 出力用ディレクトリ
  std::string output_dir = argv[2]; 
  std::filesystem::create_directories(output_dir);

  bool disable_transform = pcl::console::find_switch (argc, argv, "--NT");
  float voxel_resolution = 0.008f;
  bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
  if (voxel_res_specified)
    pcl::console::parse (argc, argv, "-v", voxel_resolution);

  float seed_resolution = 0.1f;
  bool seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
  if (seed_res_specified)
    pcl::console::parse (argc, argv, "-s", seed_resolution);

  float color_importance = 0.2f;
  if (pcl::console::find_switch (argc, argv, "-c"))
    pcl::console::parse (argc, argv, "-c", color_importance);

  float spatial_importance = 0.4f;
  if (pcl::console::find_switch (argc, argv, "-z"))
    pcl::console::parse (argc, argv, "-z", spatial_importance);

  float normal_importance = 1.0f;
  if (pcl::console::find_switch (argc, argv, "-n"))
    pcl::console::parse (argc, argv, "-n", normal_importance);

  std::ofstream yaml_file(output_dir + "/parameters.yaml"); // 出力用ディレクトリを指定
  yaml_file << "input_pcd: " << argv[1] << std::endl;
  yaml_file << "NT: " << (disable_transform ? "true" : "false") << std::endl;
  yaml_file << "v: " << voxel_resolution << std::endl;
  yaml_file << "s: " << seed_resolution << std::endl;
  yaml_file << "c: " << color_importance << std::endl;
  yaml_file << "z: " << spatial_importance << std::endl;
  yaml_file << "n: " << normal_importance << std::endl;
  yaml_file.close();

  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  if (disable_transform) super.setUseSingleCameraTransform (false);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;
  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

  // スーパーボクセルごとにファイルを保存し、インデックス番号を記録
  int index = 0;
  std::map<std::uint32_t, int> label_to_index_map;
  for (const auto& kv : supervoxel_clusters)
  {
    std::string output_path = output_dir + "/"  + std::to_string(index) + ".pcd";
    pcl::io::savePCDFileBinary(output_path, *(kv.second->voxels_));
    label_to_index_map[kv.first] = index; // スーパーボクセルのラベルとインデックスの対応を記録
    index++;
  }

  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();

  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
  // 隣接関係を保存するためのファイルをオープン
  std::ofstream adjacency_file(output_dir + "/supervoxel_adjacency.txt"); // 出力用ディレクトリを指定
  // 隣接ペアをファイルに保存
  for (const auto& pair : supervoxel_adjacency)
  {
    int supervoxel_index = label_to_index_map[pair.first];
    int neighbor_index = label_to_index_map[pair.second];
    
    adjacency_file << supervoxel_index << ", " << neighbor_index << std::endl;
  }
  adjacency_file.close();

  return (0);
}
