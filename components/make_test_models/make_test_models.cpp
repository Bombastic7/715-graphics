#include <random>
#include <string>
#include <map>
#include <cmath>
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "parse.h"



void insert_cube_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, Eigen::Affine3f& t, int seed) {
  std::mt19937 g(seed);
  std::uniform_real_distribution<> pick_coords(-1, 1);
  std::uniform_int_distribution<> pick_side(0, 5);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

  for(int i=0; i<1e4; i++) {
    int side = pick_side(g);
    float c1 = pick_coords(g);
    float c2 = pick_coords(g);
    
    if(side == 0) cloud->push_back(pcl::PointXYZ(c1, 1, c2));
    if(side == 1) cloud->push_back(pcl::PointXYZ(c1, c2, -1));
    if(side == 2) cloud->push_back(pcl::PointXYZ(1, c1, c2));
    if(side == 3) cloud->push_back(pcl::PointXYZ(c1, c2, 1));
    if(side == 4) cloud->push_back(pcl::PointXYZ(-1, c1, c2));
    if(side == 5) cloud->push_back(pcl::PointXYZ(c1, -1, c2));
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud, *cloud2, t);

  *cloud_out += *cloud2;
}


void add_gaussian_noise(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double sd, int seed = 1) {
  std::mt19937 g(seed);
  std::normal_distribution<> dst(0, sd);
  
  for(int i=0; i<cloud->size(); i++) {
    cloud->points[i].x += dst(g);
    cloud->points[i].y += dst(g);
    cloud->points[i].z += dst(g);
  }
}



// Make a random point cloud, modeling a 2*2*2 cube with a circular hole in one face.

int main(int argc, char* argv[]) {


  // corners : XYZ
  // 0 : -1 1 -1
  // 1 : 1 1 -1
  // 2 : 1 1 1
  // 3 : -1 1 1
  // 4 : -1 -1 -1
  // 5 : 1 -1 -1
  // 6 : 1 -1 1
  // 7 : -1 -1 1
  
  // c = 0 0 -1
  // r = 1
  
  if(argc == 1) {
    std::cout << "Need config file\n";
    return 1;
  }
  
  bool add_noise = false;
  bool remove_ball = false;
  std::string outputfile = "out.ply";
  
  std::map<std::string, std::string> param_map;
	if (parse_config_file(argv[1], param_map) == -1) {
    std::cout << "Could not open config file\n";
    return 1;
  }
  
  try_parse_param(param_map, "add_noise", add_noise);
  try_parse_param(param_map, "remove_ball", remove_ball);
  try_parse_param(param_map, "outputfile", outputfile);

  std::cout << "add_noise = " << add_noise << "\n";
  std::cout << "remove_ball = " << remove_ball << "\n";
  std::cout << "outputfile = " << outputfile << "\n";
  

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  
  Eigen::Affine3f t = Eigen::Affine3f::Identity();
  
  insert_cube_pointcloud(cloud, t, 1);
  
  t.translation() << 5, 0, 0;
  insert_cube_pointcloud(cloud, t, 1);
  
  t.translation() << 0, 0, 5;
  insert_cube_pointcloud(cloud, t, 1);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  
  if(remove_ball) {
    for(int i=0; i<cloud->size(); i++) {
      pcl::PointXYZ p = cloud->points[i];
      float sqdst = pow(p.x, 2) + pow(p.y, 2) + pow(p.z + 1, 2);
      if(sqrt(sqdst) <= 0.75)
        continue;
      cloud2->push_back(p);
    }
  }
  else 
    cloud2 = cloud;



  if(add_noise)
    add_gaussian_noise(cloud2, 0.1);
  
  
  pcl::io::savePLYFile("outputfile.ply", *cloud2);

  pcl::visualization::PCLVisualizer viewer(std::string("PCLVisualizer"));
  viewer.addPointCloud<pcl::PointXYZ>(cloud2);
  
  while (!viewer.wasStopped()) {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
}
