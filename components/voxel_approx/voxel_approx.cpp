#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>

#include "common.h"


template<typename PointT, typename PointU>
void make_voxel_approx(typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointU>::Ptr voxel_cloud, float voxel_size) {
  
  std::map<std::tuple<int,int,int>, int> voxel_counts;
  
  for(int p=0; p<cloud->size(); p++) {
    int x = cloud->points[p].x / voxel_size; //What is the data type of position data? Need to cast to float?
    int y = cloud->points[p].y / voxel_size;
    int z = cloud->points[p].z / voxel_size;
    
    std::tuple<int,int,int> v_ent = std::make_tuple(x,y,z);
    
    voxel_counts[v_ent] += 1;
  }
  
  voxel_cloud->points.clear();
  
  for(auto it = voxel_counts.begin(); it != voxel_counts.end(); ++it) {
    int x = std::get<0>(it->first);    
    int y = std::get<1>(it->first);    
    int z = std::get<2>(it->first);
    
    float xc = (x + 0.5) * voxel_size;
    float yc = (y + 0.5) * voxel_size;
    float zc = (z + 0.5) * voxel_size;
    
    voxel_cloud->points.emplace_back();
    voxel_cloud->points.back().x = xc;
    voxel_cloud->points.back().y = yc;
    voxel_cloud->points.back().z = zc;
  }
  
  voxel_cloud->width = voxel_cloud->points.size();
  voxel_cloud->height = 1;
  voxel_cloud->is_dense = true;
}







int main(int argc, char* argv[]) {
  if(argc == 1) {
	  std::cout << "USAGE: " << argv[0] << " [-i inputfile] [-w voxel_width]\n";
	  return 1;
  }

  std::string input_file;
  float voxel_width = 0.1;
  
  pcl::console::parse_argument(argc, argv, "-i", input_file);
  pcl::console::parse_argument(argc, argv, "-w", voxel_width);
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  
  if(load_pcd_ply<pcl::PointXYZRGBNormal>(input_file, cloud) == -1)
    return -1;
  
  make_voxel_approx<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>(cloud, voxel_cloud, voxel_width);
  
  for(int p=0; p<voxel_cloud->size(); p++) {
    voxel_cloud->points[p].r = 255;
    voxel_cloud->points[p].g = 255;
    voxel_cloud->points[p].b = 255;
  }
  
  pcl::visualization::PCLVisualizer viewer(std::string("PCLVisualizer: ") + input_file);
  viewer.addPointCloud<pcl::PointXYZRGBNormal>(voxel_cloud);
  
  while (!viewer.wasStopped()) {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
}
