#include <algorithm>
#include <iostream>
#include <vector>
#include <pcl/common/io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "common.h"


template<typename PointT>
void segment_next_plane(  typename pcl::PointCloud<PointT>::Ptr cloud, 
                          pcl::PointIndices::Ptr valid_indices, 
                          pcl::PointIndices::Ptr plane_indices,
                          pcl::ModelCoefficients::Ptr plane_coeffs) {
  
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  
  seg.setInputCloud (cloud);
  seg.setIndices(valid_indices);
  seg.segment (*plane_indices, *plane_coeffs);
}


template<typename PointT>
void make_plane_clustering( typename pcl::PointCloud<PointT>::Ptr cloud,
                            std::vector<pcl::PointIndices::Ptr>& cluster_indices,
                            std::vector<pcl::ModelCoefficients::Ptr>& cluster_coeffs) {

  pcl::PointIndices::Ptr valid_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
  
  for(int i=0; i<cloud->size(); i++)
    valid_indices->indices.push_back(i);
  
  while(valid_indices->indices.size() > 2) {
    std::cout << "Valid indices: " << valid_indices->indices.size() << "\n";
    pcl::PointIndices::Ptr plane_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coeffs = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
  
    segment_next_plane<PointT>(cloud, valid_indices, plane_indices, plane_coeffs);
    cluster_indices.push_back(plane_indices);
    cluster_coeffs.push_back(plane_coeffs);
    
    pcl::PointIndices::Ptr new_valid_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
    
    std::sort(plane_indices->indices.begin(), plane_indices->indices.end()); //Are the returned indices sorted?
    std::set_difference(valid_indices->indices.begin(), valid_indices->indices.end(), 
                        plane_indices->indices.begin(), plane_indices->indices.end(), 
                        std::back_inserter(new_valid_indices->indices));
    
    valid_indices = new_valid_indices;
  }
}


template<typename PointT>
void visualize_plane_segments(  typename pcl::PointCloud<PointT>::Ptr cloud,
                                std::vector<pcl::PointIndices::Ptr>& cluster_indices) {

  std::vector<float> color_scale_values;
  pcl::PointCloud<pcl::RGB>::Ptr cluster_rgb = pcl::PointCloud<pcl::RGB>::Ptr(new pcl::PointCloud<pcl::RGB>);
  cluster_rgb->points.resize(cluster_indices.size());
  
  make_random_equidistant_range_assignment<float>(cluster_indices.size(), color_scale_values);
  make_rgb_scale<float, pcl::RGB>(color_scale_values, cluster_rgb, COLOR_MAP_RAINBOW);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::copyPointCloud/*<PointT, pcl::PointXYZRGB>*/(*cloud, *cloud_colored);

  for(int i=0; i<cluster_indices.size(); i++) {
    for(int j=0; j<cluster_indices[i]->indices.size(); j++) {
      cloud_colored->points[cluster_indices[i]->indices[j]].r = cluster_rgb->points[i].r;
      cloud_colored->points[cluster_indices[i]->indices[j]].g = cluster_rgb->points[i].g;
      cloud_colored->points[cluster_indices[i]->indices[j]].b = cluster_rgb->points[i].b;
    }
  }
  
  pcl::visualization::PCLVisualizer viewer(std::string("PCLVisualizer: "));
  viewer.addPointCloud<pcl::PointXYZRGB>(cloud_colored);
  
  while (!viewer.wasStopped()) {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   }
}


int
main (int argc, char** argv)
{
  if(argc == 1) {
    std::cout << "Need pcd/ply file argument.\n";
    return 1;
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  if(load_pcd_ply<pcl::PointXYZ>(argv[1], cloud) == -1)
    return 1;

  std::cout << "Input cloud points: " << cloud->size() << "\n";

  std::vector<pcl::PointIndices::Ptr> cluster_indices;
  std::vector<pcl::ModelCoefficients::Ptr> cluster_coeffs;
  
  make_plane_clustering<pcl::PointXYZ>(cloud, cluster_indices, cluster_coeffs);
  
  std::cout << "Clusters: " << cluster_indices.size() << "\n";
  
  visualize_plane_segments<pcl::PointXYZ>(cloud, cluster_indices);
  
  return (0);
}


