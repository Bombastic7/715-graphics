#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/console/time.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/extract_indices.h>


int
main (int argc, char** argv)
{
  pcl::console::TicToc tictoc;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  
  
  if(argc == 1) {
	std::cout << "USAGE: " << argv[0] << " file.[ply|pcd]" << std::endl;
	return 1;
  }
  
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  
  if(filenames.size() > 0) {
	  if(pcl::io::loadPCDFile(argv[filenames[0]], *cloud) == -1) {
		  PCL_ERROR("Bad file name\n");
		  return 1;
	  }
  }
  else {
	  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
	  if(filenames.size() == 0) {
		  PCL_ERROR("No ply/pcd file given");
		  return 1;
	  }
	  if(pcl::io::loadPLYFile(argv[filenames[0]], *cloud) == -1) {
		  PCL_ERROR("Bad file name\n");
		  return 1;
	  }
  }

  std::cout << "Loaded. Size: " << cloud->width << " * " << cloud->height << " = " << cloud->width * cloud->height << std::endl;

  tictoc.tic();
  
  std::cout << "Computing Normals. " << std::flush;
  
  // compute normals
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);

  std::cout << tictoc.toc() / 1000 << "s\n";

  

/*
  std::cout << "filtering indices\n";
  // filtering indices (probably don't need this)
  pcl::IndicesPtr indices(new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*indices);
*/

  
  std::cout << "Region Growing. " << std::flush;
  
  // region growing algorithm
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(cloud);
  //reg.setIndices (indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract(clusters);

  std::cout << tictoc.toc() / 1000 << "s\n";
  
  
  std::cout << "\n\nDone.\n";
  std::cout << "Clusters: " << clusters.size() << "\n";

  int i = 0;
  for (i = 0; i < clusters.size(); i++) {
	  // Extract point cloud from the indices
	  pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);

	  std::vector <pcl::PointIndices> currCluster;
	  currCluster.push_back(clusters[i]);
	  pcl::ExtractIndices<pcl::PointXYZ> eifilter(true);
	  eifilter.setInputCloud(cloud);

	  eifilter.setIndices(clusters[i].indices); //BUG: the method aguments are of the incorrect type
	  eifilter.filter(*newCloud);
	  /*
	  int j = 0;
	  for (j = 0; j < clusters[i].indices.size(); j++) {
		  //newCloud[j] =
	  }

	  // compute normals 
	  // TODO: use the normals computed before
	  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	  pcl::PointCloud <pcl::Normal>::Ptr newNormals(new pcl::PointCloud <pcl::Normal>);
	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	  normal_estimator.setSearchMethod(tree);
	  normal_estimator.setInputCloud(cloud);
	  normal_estimator.setKSearch(50);
	  normal_estimator.compute(*newNormals);

	  // Concatenate the XYZ and normal fields*
	  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	  pcl::concatenateFields(*cloud, *newNormals, *cloud_with_normals);
	  //* cloud_with_normals = cloud + normals
	  */
  }


  if (clusters.size() == 0) {
	  std::cout << "Exiting as no clusters found.\n";
	  return (0);
  }

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
  pcl::visualization::CloudViewer viewer(std::string("Cluster Viewer: ") + argv[filenames[0]]);
  viewer.showCloud(colored_cloud);
  
  while (!viewer.wasStopped()) {}

  // Write to ply file
 
  return (0);
}
