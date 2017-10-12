#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <pcl/common/io.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "common.h"



template<typename PointT>
class PlaneSegmentation {
  public:
  
  PlaneSegmentation(typename pcl::PointCloud<PointT>::Ptr cloud, double dist_th, double samples_max_dist, double eps_angle, double norm_est_rad) :
    cloud_normals_(new pcl::PointCloud<pcl::Normal>),
    valid_indices_(new pcl::PointIndices),
    tree_(new typename pcl::search::KdTree<PointT>),
    cloud_(cloud),
    dist_th_(dist_th),
    samples_max_dist_(samples_max_dist),
    eps_angle_(eps_angle),
    norm_est_rad_(norm_est_rad)
  {
    for(int i=0; i<cloud->size(); i++) {
      valid_indices_->indices.push_back(i);
    }
    
    pcl::NormalEstimation<PointT, pcl::Normal> normal_est;
    normal_est.setInputCloud (cloud_);
    normal_est.setSearchMethod (tree_);
    normal_est.setRadiusSearch (norm_est_rad_);
    normal_est.compute (*cloud_normals_);
    
    seg_.setOptimizeCoefficients (true);
    seg_.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setDistanceThreshold (dist_th_);
    seg_.setEpsAngle(eps_angle_);
    seg_.setSamplesMaxDist(samples_max_dist_, tree_);
    seg_.setInputCloud (cloud_);
    seg_.setIndices(valid_indices_);
    seg_.setInputNormals(cloud_normals_);
  }
  
  void run() {
    while(true) {      
      if(valid_indices_->indices.size() < 3)
        break;
      
      std::cout << "Valid indices: " << valid_indices_->indices.size() << "\n";
      pcl::PointIndices::Ptr plane_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr plane_coeffs = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
      seg_.segment (*plane_indices, *plane_coeffs);
      
      if(plane_indices->indices.size() > 0) {
        cluster_indices_.push_back(plane_indices);
        cluster_coeffs_.push_back(plane_coeffs);
        
        pcl::PointIndices::Ptr new_valid_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
      
        std::sort(plane_indices->indices.begin(), plane_indices->indices.end()); //Are the returned indices sorted?
        std::set_difference(valid_indices_->indices.begin(), valid_indices_->indices.end(), 
                            plane_indices->indices.begin(), plane_indices->indices.end(), 
                            std::back_inserter(new_valid_indices->indices));
      
        valid_indices_ = new_valid_indices;
        seg_.setIndices(valid_indices_);
      }
      else
        break;

    }
  }
  
  
  void project_points() {
	for(int cl=0; cl<cluster_indices_.size(); cl++) {
		
		float a = cluster_coeffs_[cl]->values[0];
		float b = cluster_coeffs_[cl]->values[1];
		float c = cluster_coeffs_[cl]->values[2];
		float d = cluster_coeffs_[cl]->values[3];
		float s = pow(a, 2) + pow(b, 2) + pow(c, 2);
		
		
		for(int i=0; i<cluster_indices_[cl]->indices.size(); i++) {
			int p = cluster_indices_[cl]->indices[i];
			float u = cloud_->points[p].x;
			float v = cloud_->points[p].y;
			float w = cloud_->points[p].z;
			
			float t = (a*u + b*v + c*w + d) / s;
			float x0 = u - a * t;
			float y0 = v - b * t;
			float z0 = w - c * t;
			
			cloud_->points[p].x = x0;
			cloud_->points[p].y = y0;
			cloud_->points[p].z = z0;
		}
	}
  }
  
  
  void visualize() {
    std::vector<float> color_scale_values;
    pcl::PointCloud<pcl::RGB>::Ptr cluster_rgb = pcl::PointCloud<pcl::RGB>::Ptr(new pcl::PointCloud<pcl::RGB>);
    cluster_rgb->points.resize(cluster_indices_.size());
    
    make_random_equidistant_range_assignment<float>(cluster_indices_.size(), color_scale_values);
    make_rgb_scale<float, pcl::RGB>(color_scale_values, cluster_rgb, COLOR_MAP_RAINBOW);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_colored = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    pcl::copyPointCloud(*cloud_, *cloud_colored);
    
    for(int i=0; i<cloud_->size(); i++) {
      cloud_colored->points[i].r = 255;
      cloud_colored->points[i].g = 255;
      cloud_colored->points[i].b = 255;
    }
  
    for(int i=0; i<cluster_indices_.size(); i++) {
      std::cout << "Cluster " << i << " color: " << (int)(cluster_rgb->points[i].r) << ", " << (int)(cluster_rgb->points[i].g) << ", " << (int)(cluster_rgb->points[i].b) << "\n";
      for(int j=0; j<cluster_indices_[i]->indices.size(); j++) {
        cloud_colored->points[cluster_indices_[i]->indices[j]].r = cluster_rgb->points[i].r;
        cloud_colored->points[cluster_indices_[i]->indices[j]].g = cluster_rgb->points[i].g;
        cloud_colored->points[cluster_indices_[i]->indices[j]].b = cluster_rgb->points[i].b;
      }
    }
    
    pcl::visualization::PCLVisualizer viewer(std::string("PCLVisualizer"));
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud_colored);
    
    while (!viewer.wasStopped()) {
      viewer.spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
     }
  }
  
    
  
  
  
  
  private:
  
  std::vector<pcl::PointIndices::Ptr> cluster_indices_;
  std::vector<pcl::ModelCoefficients::Ptr> cluster_coeffs_;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
  pcl::PointIndices::Ptr valid_indices_;
  typename pcl::search::KdTree<PointT>::Ptr tree_;
  typename pcl::PointCloud<PointT>::Ptr cloud_;
  double dist_th_;
  double samples_max_dist_;
  double eps_angle_;
  double norm_est_rad_;
  
};



/*

#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <queue>
#include <Eigen/Core>

struct CompIndicesByCurvature {
  CompIndicesByCurvature(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals):
    cloud_normals_ = cloud_normals
  {}
  
  bool operator()(int a, int b) {
    return cloud_normals->points[a].curvature < cloud_normals->points[b].curvature;
  }
  
  private:
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
};


template<typename PointT>
void make_plane_clustering2( typename pcl::PointCloud<PointT>::Ptr cloud,
                            std::vector<pcl::PointIndices::Ptr>& cluster_indices,
                            std::vector<pcl::ModelCoefficients::Ptr>& cluster_coeffs) {

  const double Reg_Growing_Radius = 0.1;
  const int Reg_Growing_k = 20;
  const float Reg_Growing_Angle_Th = 0.05235987755; //3 degrees.
  
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
  
  pcl::NormalEstimation<PointT, pcl::Normal> normal_est;
  normal_est.setInputCloud (cloud);

  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  normal_est.setSearchMethod (tree);
  normal_est.setRadiusSearch (rad);
  normal_est.compute (*cloud_normals);

  
  int cur_cluster = 0;
  int assigned_points = 0;
  std::vector<int> cluster_assignment(cloud->size());
  std::priority_queue<int, std::vector<int>, CompIndicesByCurvature> curv_sorted(CompIndicesByCurvature(cloud_normals));
  std::deque seed_queue;
  std::vector<int> centroids;
  
  for(int i=0; i<cloud->size(); i++) {
    curv_sorted.push(i);
  }
  
  while(assigned_points < cloud->size()) {
    cur_cluster++;
    
    int centroid_point;
    do { 
      centroid_point = curv_sorted.pop();
    } while(cluster_assignment[centroid_point] != 0);
    
    Eigen::Vector3f centroid_normal = cloud->points[centroid_point].getNormalVector3fMap();
    
    seed_queue.push(centroid_point);
    centroids.push_back(centroid_point);
    
    while(!seed_queue.empty()) {
      int p = seed_queue.pop();
      cluster_assignment[p] = cur_cluster;
      assigned_points++;
      
      std::vector<int> nghbr_indices(Reg_Growing_k);
      std::vector<float> nghbr_sqdst(Reg_Growing_k);
      
      int ngbhrs = tree.radiusSearch(cloud->points[p], Reg_Growing_Radius, nghbr_indices, nghbr_sqdst, Reg_Growing_k);
      
      for(int i=0; i<nghbrs; i++) {
        int q = nghbr_indices[i];
        
        if(cluster_assignment[q] != 0)
          continue;
        
        Eigen::Vector3f q_n = cloud->points[q].getNormalVector3fMap();
        
        float angle = acos(centroid_normal.dot(q_n));
        
        if(angle <= Reg_Growing_Angle_Th) {
          seed_queue.push(q);
        }
      }
    }
  }
  

  assert(assigned_points == cloud->size());
  assert(centroids.size() == cur_cluster);
  
  for(int i=0; i<cur_cluster; i++) {
    pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
    cluster_indices.push_back(indices);
  }
  
  for(int i=0; i<cloud->size(); i++) {
    cluster_indices.at(cluster_assignment[i] - 1)->indices.push_back(i);
  }
  
  for(int i=0; i<cur_cluster; i++) {
    float x0 = cloud->points[centroids[i]].x;
    float y0 = cloud->points[centroids[i]].y;
    float z0 = cloud->points[centroids[i]].z;
    float a = cloud_normals->points[centroids[i]].normal[0];
    float b = cloud_normals->points[centroids[i]].normal[1];
    float c = cloud_normals->points[centroids[i]].normal[2];
    float d = - ( a * x0 + b * y0 + c * z0 );
    
    pcl::ModelCoefficients::Ptr c = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients));
    c->values[0] = a;
    c->values[0] = b;
    c->values[0] = c;
    c->values[0] = d;
    cluster_coeffs.push_back(c);
  }

}

*/



std::string g_inputfile;
double g_dist_th = 0.05;
double g_samples_max_dist = 1;
double g_eps_angle = 0.05235987755;
double g_norm_est_rad = 0.05;
bool g_project_points = false;


int
main (int argc, char** argv)
{
  if(argc == 1) {
    std::cout << "USAGE: \n"
    << "-i input file\n"
    << "-d RANSAC plane model distance threshold\n"
    << "-s RANSAC samples max distance\n"
    << "-e RANSAC Epsilon angle, max deviation in normal\n"
    << "-r normal estimation search radius\n"
    << "-p boolean, project points onto planes\n";
    return 1;
  }
  
  std::string input_file;
  float dist_th = 0.01;
  
  pcl::console::parse_argument(argc, argv, "-i", g_inputfile);
  pcl::console::parse_argument(argc, argv, "-d", g_dist_th);
  pcl::console::parse_argument(argc, argv, "-s", g_samples_max_dist);
  pcl::console::parse_argument(argc, argv, "-e", g_eps_angle);
  pcl::console::parse_argument(argc, argv, "-r", g_norm_est_rad);
  pcl::console::parse_argument(argc, argv, "-p", g_project_points);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  
  if(load_pcd_ply<pcl::PointXYZ>(g_inputfile, cloud) == -1)
    return 1;
    

  PlaneSegmentation<pcl::PointXYZ> ps(cloud, g_dist_th, g_samples_max_dist, g_eps_angle, g_norm_est_rad);
  
  ps.run();
  
  if(g_project_points)
	ps.project_points();

  ps.visualize();
  
  return (0);
}


