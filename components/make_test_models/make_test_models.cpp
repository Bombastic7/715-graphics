#include <random>
#include <string>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>


#include <pcl/visualization/pcl_visualizer.h>





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
  

  std::mt19937 g;
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
  
  
  for(int i=0; i<cloud->size(); i++) {
    pcl::PointXYZ p = cloud->points[i];
    float sqdst = pow(p.x, 2) + pow(p.y, 2) + pow(p.z + 1, 2);
    if(sqrt(sqdst) <= 0.75)
      continue;
    cloud2->push_back(p);
  }

  pcl::io::savePLYFile("CubeWithHole.ply", *cloud2);
  

  pcl::visualization::PCLVisualizer viewer(std::string("PCLVisualizer"));
  viewer.addPointCloud<pcl::PointXYZ>(cloud2);
  
  while (!viewer.wasStopped()) {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
}
