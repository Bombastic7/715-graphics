#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <boost/thread/thread.hpp>
#include <pcl/common/eigen.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/point_cloud_color_handlers.h>
//#include <pcl/visualization/point_cloud_geometry_handlers.h>

#include "common.h"


std::string g_infile, g_outfile;
float g_norm_est_radius = 0.03;


void print_help(int argc, char* argv[]) {
	std::cout << "USAGE: " << argv[0] << "[-o outfile] [-i infile] [-r radius]" << std::endl;
	std::cout << "Input file either PLY or PCD format. Output file is PLY." << std::endl;
}



bool parse_input(int argc, char* argv[]) {

  pcl::console::parse_argument(argc, argv, "-i", g_infile);
  pcl::console::parse_argument(argc, argv, "-o", g_outfile);
  pcl::console::parse_argument(argc, argv, "-r", g_norm_est_radius);
  
  
  if(g_infile == "") {
	  PCL_ERROR("No input file given");
	  return false;
  }

  return true;
}




template<typename PointP, typename PointN>
int compute_normals(  typename pcl::PointCloud<PointP>::Ptr cloud, 
                      typename pcl::PointCloud<PointN>::Ptr cloud_normals, 
                      float rad) {
  pcl::NormalEstimation<PointP, PointN> normal_est;
  normal_est.setInputCloud (cloud);

  typename pcl::search::KdTree<PointP>::Ptr tree (new pcl::search::KdTree<PointP> ());
  normal_est.setSearchMethod (tree);

  normal_est.setRadiusSearch (rad);

  normal_est.compute (*cloud_normals);
	return 0;
}



template<typename PointType>
double
computeCloudResolution (typename pcl::PointCloud<PointType>::Ptr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}


float g_dist_threshold = 0.01;




template<typename PointT>
int compute_position_deviance(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<float>& deviance, float dist_threshold) {
  //Position deviance
  //Compute best plane model fit using RANSAC.
  //Options: model distance threshold.
  
  pcl::SACSegmentation<PointT> seg;
  pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(dist_threshold);
  
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coeffs);
  
  //... compute deviance of each point wrt computed plane.
}



template<typename PointT>
int compute_normal_deviance(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<float>& deviance, float rad1, float rad2) {

  pcl::PointCloud<pcl::Normal>::Ptr normals_1(new pcl::PointCloud<pcl::Normal>), normals_2(new pcl::PointCloud<pcl::Normal>);
  
  compute_normals<PointT, pcl::Normal>(cloud, normals_1, rad1);
  compute_normals<PointT, pcl::Normal>(cloud, normals_2, rad2);

  deviance.resize(cloud->points.size());
  
  for(int i=0; i<cloud->points.size(); i++) {
    Eigen::Vector3f n1 = normals_1->points[i].getNormalVector3fMap();
    Eigen::Vector3f n2 = normals_2->points[i].getNormalVector3fMap();
    
    deviance[i] = n1.dot(n2);
  }
  
  return 0;
}


//PointT expected to have computed normal/curvature component, xyz component.
template<typename PointT>
int compute_interest(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<float>& interest) {
  //interest = curvature + curvature deviance + position deviance + normal deviance
  

  return 0;
  
}


template<typename PointT>
int extract_interest_clusters(  typename pcl::PointCloud<PointT>::Ptr cloud, 
                                std::vector<float>& interest_val, 
                                float interest_th,
                                float cluster_tolerance,
                                std::vector<pcl::PointIndices>& cluster_indices)
{
	
  pcl::IndicesPtr kept_indices = pcl::IndicesPtr(new std::vector<int>);
  
  for(int i=0; i<cloud->size(); i++)
    if(interest_val[i] >= interest_th)
      kept_indices->push_back(i);
  
  pcl::search::KdTree<PointT> tree;
  tree.setInputCloud (cloud);
  
  pcl::EuclideanClusterExtraction<PointT> ec;
  
  ec.setSearchMethod(tree);
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(cloud->size());
  ec.setClusterTolerance(0.3);
  ec.setInputCloud(cloud);
  ec.setIndices(kept_indices);
  ec.extract(cluster_indices);

}




int main(int argc, char* argv[]) {
  if(argc == 1) {
	  print_help(argc, argv);
	  return 1;
  }

  if(!parse_input(argc, argv))
    return 1;
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  
  if(load_pcd_ply<pcl::PointXYZRGBNormal>(g_infile, cloud) == -1)
    return -1;


  std::vector<float> ndeviance;
  
  compute_normal_deviance<pcl::PointXYZRGBNormal>(cloud, ndeviance, 0.03, 0.3);
  
  //make_rgb_scale<float, pcl::PointXYZRGBNormal>(ndeviance, cloud, COLOR_MAP_COOLWARM);

  //std::vector<pcl::PointIndices> cluster_indices;
  //extract_interest_clusters(cloud, ndeviance, 0.2
  
  pcl::IndicesPtr kept_indices = pcl::IndicesPtr(new std::vector<int>);
  
  for(int i=0; i<cloud->size(); i++) {
    if(ndeviance[i] >= 0.2)
      kept_indices->push_back(i);
  }
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::ExtractIndices<pcl::PointXYZRGBNormal> ei_filter(true);
  ei_filter.setInputCloud(cloud);
  ei_filter.setIndices(kept_indices);
  ei_filter.filter(*cloud_filtered);
  
  pcl::visualization::PCLVisualizer viewer(std::string("PCLVisualizer: ") + g_infile);
  viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud_filtered);
  
  while (!viewer.wasStopped()) {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
}
