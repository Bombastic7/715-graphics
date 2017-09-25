#include <iostream>
#include <string>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>

#include "common.h"


std::string g_infile, g_outfile;


void print_help(int argc, char* argv[]) {
	std::cout << "USAGE: " << argv[0] << "[-o outfile] [-i infile]" << std::endl;
	std::cout << "Input file either PLY or PCD format. Output file is PLY." << std::endl;
}



bool parse_input(int argc, char* argv[]) {

  pcl::console::parse_argument(argc, argv, "-i", g_infile);
  pcl::console::parse_argument(argc, argv, "-o", g_outfile);
  
  
  if(g_infile == "") {
	  PCL_ERROR("No input file given");
	  return false;
  }

  return true;
}


template<typename PointT>
int load_file(std::string const& f, typename pcl::PointCloud<PointT>::Ptr cloud) {
  int res;
  if(ends_with(f, ".pcd") && (res = pcl::io::loadPCDFile(f, *cloud)) == 0) {}
  else if(ends_with(f, ".ply") && (res = pcl::io::loadPLYFile(f, *cloud)) == 0) {}
  else {
	  PCL_ERROR("Bad input file. %i\n", res);
      return -1;
  }
  return 0;
}


int main(int argc, char* argv[]) {
  if(argc == 1) {
	  print_help(argc, argv);
	  return 1;
  }

  if(!parse_input(argc, argv))
    return 1;
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz (new pcl::PointCloud<pcl::PointXYZ>);
  
  if(load_file<pcl::PointXYZRGBNormal>(g_infile, cloud) == -1)
	return -1;

/*
  cloud->points.resize(cloudxyz->points.size());
  cloud->width = cloudxyz->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
  
  for(int i=0; i<cloudxyz->points.size(); i++) {
	  cloud->points[i].x = cloudxyz->points[i].x;
	  cloud->points[i].y = cloudxyz->points[i].y;
	  cloud->points[i].z = cloudxyz->points[i].z;
  }
*/

  pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> normal_est;
  normal_est.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
  normal_est.setSearchMethod (tree);

  normal_est.setRadiusSearch (0.03);

  normal_est.compute (*cloud);
  
  
  
  float curv_min, curv_max;
  curv_min = curv_max = cloud->points[0].curvature;
  
  for(auto it = cloud->points.begin(); it != cloud->points.end(); ++it) {
	  if(it->curvature < curv_min)
		curv_min = it->curvature;
	  if(it->curvature > curv_max)
	    curv_max = it->curvature;
  }
  
  for(auto it = cloud->points.begin(); it != cloud->points.end(); ++it) {
    int h = int((curv_max - it->curvature) / (curv_max - curv_min) * 256 * 6);
    int x = h % 0x100;

    int r = 0, g = 0, b = 0;
    switch (h / 256)
    {
      case 0: r = 255; g = x;       break;
      case 1: g = 255; r = 255 - x; break;
      case 2: g = 255; b = x;       break;
      case 3: b = 255; g = 255 - x; break;
      case 4: b = 255; r = x;       break;
      case 5: r = 255; b = 255 - x; break;
    }
    it->r = r;
    it->g = g;
    it->b = b;
  }
  
  

  pcl::visualization::PCLVisualizer viewer(std::string("PCLVisualizer: ") + g_infile);
  
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> rgb_handler(cloud);
  pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PointXYZRGBNormal> xyz_handler(cloud);
  
  //viewer.addPointCloudNormals<pcl::PointNormal>(cloud);
  viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud, rgb_handler, xyz_handler);
  //viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud, 100, 0.02f, "normals");
  
  while (!viewer.wasStopped()) {
	 viewer.spinOnce (100);
     boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
}
