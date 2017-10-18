#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include <cassert>
#include <cmath>
#include <Eigen/Core>
/*
#include <pcl/common/centroid.h>
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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
*/
#include "common.h"
#include "parse.h"
#include "face_graph.hpp"

/*
struct GeomDescriptors {
  double bb_x, bb_y, bb_z;
  double o_x, o_y, o_z;
};

template<typename PointT>
struct FaceCluster {
  pcl::PointCloud<PointT>::Ptr cloud;
  GeomDescriptors geomdesc;
  PointT centroid;
};

struct FaceEdgeProps {
  float centroid_relpos;
};


  

template<typename PointT>
class FaceGraphSegmentor {
  public:
  
  FaceGraphSegmentor( typename pcl::PointCloud<PointT>::Ptr cloud, 
                      double p_norm_est_k,
                      double p_pc_dist_th, 
                      double p_pc_samples_max_dist, 
                      double p_pc_eps_angle, 
                      int p_pc_min_points,
                      double p_fc_maxrad,
                      double p_adj_sz,
                      int p_adj_k,
					  bool p_proj) :
    cloud_normals_(new pcl::PointCloud<pcl::Normal>),
    kdtree_(new typename pcl::search::KdTree<PointT>),
    cloud_(cloud),
    p_norm_est_k_(p_norm_est_k),
    p_pc_dist_th_(p_pc_dist_th),
    p_pc_samples_max_dist_(p_pc_samples_max_dist),
    p_pc_eps_angle_(p_pc_eps_angle),
	p_pc_min_points_(p_pc_min_points),
    p_fc_maxrad_(p_fc_maxrad),
    p_adj_sz_(p_adj_sz),
    p_adj_k_(p_adj_k),
	p_proj_(p_proj)
  {
  }
  

   
  int run() {
    compute_normals();
    segment_planes();
    segment_faces();
    compute_adjacency();

	if (p_proj_)
		project_points();
	compute_geom_descriptors();
	return 0;
  }
  
  
    
  void visualize(int use_color_map = COLOR_MAP_RAINBOW, int use_seed = 12345) {
    std::vector<float> color_scale_values;
    pcl::PointCloud<pcl::RGB>::Ptr cluster_rgb = pcl::PointCloud<pcl::RGB>::Ptr(new pcl::PointCloud<pcl::RGB>);
    cluster_rgb->points.resize(cluster_indices_.size());
    
    make_random_equidistant_range_assignment<float>(cluster_indices_.size(), color_scale_values, use_seed);
    make_rgb_scale<float, pcl::RGB>(color_scale_values, cluster_rgb, use_color_map);
    
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
  
    
  
  protected:
  
  void compute_normals() {
    pcl::NormalEstimation<PointT, pcl::Normal> normal_est;
    normal_est.setInputCloud (cloud_);
    normal_est.setSearchMethod (kdtree_);
    
    //normal_est.setRadiusSearch (p_norm_est_rad_);
    normal_est.setKSearch(p_norm_est_k_);
    normal_est.compute (*cloud_normals_);
  }
  
  void segment_planes() {
    
    pcl::PointIndices::Ptr valid_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
  
    for(int i=0; i<cloud_->size(); i++) {
      valid_indices->indices.push_back(i);
    }
    
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;    
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (p_pc_dist_th_);
    seg.setEpsAngle(p_pc_eps_angle_);
    seg.setSamplesMaxDist(p_pc_samples_max_dist_, kdtree_);
    seg.setInputCloud (cloud_);
    seg.setInputNormals(cloud_normals_);
    seg.setIndices(valid_indices);
    
    while(true) {      
      if(valid_indices->indices.size() < 3)
        break;
      
      std::cout << "Valid indices: " << valid_indices->indices.size() << "\n";
      pcl::PointIndices::Ptr plane_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr plane_coeffs = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
      seg.segment (*plane_indices, *plane_coeffs);
      
      if(plane_indices->indices.size() > p_pc_min_points_) {
        cluster_indices_.push_back(plane_indices);
        cluster_coeffs_.push_back(plane_coeffs);
        
        pcl::PointIndices::Ptr new_valid_indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
      
        std::sort(plane_indices->indices.begin(), plane_indices->indices.end()); //Are the returned indices sorted?
        std::set_difference(valid_indices->indices.begin(), valid_indices->indices.end(), 
                            plane_indices->indices.begin(), plane_indices->indices.end(), 
                            std::back_inserter(new_valid_indices->indices));
      
        valid_indices = new_valid_indices;
        seg.setIndices(valid_indices);
      }
      else
        break;

    }
  }
  
  void segment_faces() {
	  std::vector<pcl::PointIndices::Ptr> new_cluster_indices;
	  std::vector<pcl::ModelCoefficients::Ptr> new_cluster_coeffs;
	  
	  pcl::EuclideanClusterExtraction<PointT> ec;
	  ec.setMinClusterSize(50);
	  ec.setMaxClusterSize(1000000);
	  ec.setClusterTolerance(p_fc_maxrad_);
	  ec.setSearchMethod(kdtree_);
	  ec.setInputCloud(cloud_);
	   
	for(int cl=0; cl<cluster_indices_.size(); cl++) {
		std::cout << "Cluster " << cl << "\n";
		ec.setIndices (cluster_indices_[cl]);
	  
		std::vector <pcl::PointIndices> clusters;
		ec.extract(clusters);
	  
		for(int i=0; i<clusters.size(); i++) {
			  pcl::PointIndices::Ptr clusterptr = pcl::PointIndices::Ptr(new pcl::PointIndices);
			  clusterptr->indices = std::vector<int>(clusters[i].indices);
			  new_cluster_indices.push_back(clusterptr);
			  new_cluster_coeffs.push_back(cluster_coeffs_[cl]);
      }
    }
	
    cluster_indices_ = new_cluster_indices;
    cluster_coeffs_ = new_cluster_coeffs;
  }
  
  void compute_adjacency() {
    //Consider clusters A and B adjacent if there are at least p_adj_k_ points in A that are within (2*sqrt(3)*p_adj_sz_) distance of some point in B and vice versa.
    const float r = p_adj_sz_;
    
    std::vector<int> cluster_assignment = std::vector<int>(cloud_->size(), -1);
    
    for(int c=0; c<cluster_indices_.size(); c++)
      for(int i=0; i<cluster_indices_[c]->indices.size(); i++)
        cluster_assignment[cluster_indices_[c]->indices[i]] = c;
    
    
    //Divide model space into cube regions of length r. Region identified by coordinates (x,y,z).
    //One corner is (x,y,z)*r. Opposite corner is (x +- 1, y +- 1, z +- 1), add 1 if dimension >= 0, else subtract 1.
    
    //For each region, count number of points contained for each cluster.
    
    std::map<std::tuple<int,int,int>, std::map<int,int>> region_counts;
    
    for(int p=0; p<cloud_->size(); p++) {
		if (cluster_assignment[p] == -1)
			continue;

      std::tuple<int,int,int> reg = std::make_tuple(  pos_to_reg(cloud_->points[p].x, r), 
                                                      pos_to_reg(cloud_->points[p].y, r),
                                                      pos_to_reg(cloud_->points[p].z, r));
      
      if(region_counts.count(reg) == 0)
        region_counts[reg] = std::map<int,int>();
      
      region_counts[reg][cluster_assignment[p]]++;
    }
    
    //For each region R, for each cluster C in R, for each cluster D != C in R and its 26 adjacent regions, 
    //  count n = # points of C in R, m = # points of D in R+adj regions
    //  pair_counts[(C,D)] += min(n,m)
    
    std::map<std::tuple<int,int>, int> pair_counts;
    
    
    for(auto regit=region_counts.begin(); regit!=region_counts.end(); ++regit) {
      
      int x = std::get<0>(regit->first);
      int y = std::get<1>(regit->first);
      int z = std::get<2>(regit->first);
      
      std::map<int,int> super_region_counts;
      
      for(int xd=-1; xd <= 1; xd += 1) {
        for(int yd=-1; yd <= 1; yd += 1) {
          for(int zd=-1; zd <= 1; zd += 1) {
            std::tuple<int,int,int> adjreg = std::make_tuple(x + xd, y + yd, z + zd);
            
            if(region_counts.count(adjreg) == 0)
              continue;
            
            for(auto it=region_counts[adjreg].begin(); it!=region_counts[adjreg].end(); ++it) {
              super_region_counts[it->first] += it->second;
            }
          }
        }
      }
      
      for(auto it=regit->second.begin(); it!=regit->second.end(); ++it) {
        for(auto it2=super_region_counts.begin(); it2!=super_region_counts.end(); ++it2) {
          
          if(it->first == it2->first)
            continue;
          
          std::tuple<int,int> t = std::make_tuple(it->first, it2->first);
          int n = it->second < it2->second ? it->second : it2->second;
          
          pair_counts[t] += n;
        }
      }
    }
    
    adjlist_ = std::vector<std::vector<int>>(cluster_indices_.size());
    
    for(int i=0; i<cluster_indices_.size(); i++) {
      for(int j=i+1; j<cluster_indices_.size(); j++) {
        if(pair_counts[std::make_tuple(i,j)] > p_adj_k_ && pair_counts[std::make_tuple(j,i)] > p_adj_k_) {
          adjlist_[i].push_back(j);
          adjlist_[j].push_back(i);
        }
      }
    }
    
    for(int i=0; i<adjlist_.size(); i++) {
      std::cout << "Cluster " << i << ": ";
      for(int j=0; j<adjlist_[i].size(); j++) {
        std::cout << adjlist_[i][j] << " ";
      }
      std::cout << "\n";
    }
  }
  

  
  static int pos_to_reg(float x, float r) {
    if(x >= 0)
      return x / r;
    return x / r - 1;
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
  
  float face_similarity(int a, int b) {
	  assert(a >= 0 && b >= 0);
	  assert(face_geom_desc_.size() > a && face_geom_desc_.size() > b);


	  float bbox_diff = abs(face_geom_desc_[a].bb_x - face_geom_desc_[b].bb_x) +
		  abs(face_geom_desc_[a].bb_y - face_geom_desc_[b].bb_y) +
		  abs(face_geom_desc_[a].bb_z - face_geom_desc_[b].bb_z);

	  bbox_diff /= face_avg_bbox_lengths_ * 10;
	  //0 = perfect match
	  //1 = difference is one tenth of avg bbox edge sum lengths.

	  float dp = face_geom_desc_[a].o_x * face_geom_desc_[b].o_x +
		  face_geom_desc_[a].o_y * face_geom_desc_[b].o_y +
		  face_geom_desc_[a].o_z * face_geom_desc_[b].o_z;

	  if (dp > 1) dp = 1;

	  float o_diff = acos(dp) / (15.0f / 180.0f * M_PI);
	  //0 = perfect match
	  //1 = difference in orientation is 15 degrees

	  if (bbox_diff > 1) {
		  bbox_diff = 1;
	  }
	  if (o_diff > 1) {
		  o_diff = 1;
	  }

	  return 1 - bbox_diff * 0.5f - o_diff * 0.5f;
	}


	void compute_geom_descriptors() {
		for (int i = 0; i < cluster_indices_.size(); i++) {
			GeomDescriptors g;

			float x_max, x_min, y_max, y_min, z_max, z_min;
			x_max = y_max = z_max = std::numeric_limits<float>::min();
			x_min = y_min = z_min = std::numeric_limits<float>::max();

			for (int p = 0; p < cluster_indices_[i]->indices.size(); p++) {
				float x = cloud_->points[cluster_indices_[i]->indices[p]].x;
				float y = cloud_->points[cluster_indices_[i]->indices[p]].y;
				float z = cloud_->points[cluster_indices_[i]->indices[p]].z;
				assert(isfinite(x) && isfinite(y) && isfinite(z));
	
				if (x > x_max) x_max = x;
				if (y > y_max) y_max = y;
				if (z > z_max) z_max = z;
				if (x < x_min) x_min = x;
				if (y < y_min) y_min = x;
				if (z < z_min) z_min = z;
			}

			g.bb_x = x_max - x_min;
			g.bb_y = y_max - y_min;
			g.bb_z = z_max - z_min;

			g.o_x = cluster_coeffs_[i]->values[0];
			g.o_y = cluster_coeffs_[i]->values[1];
			g.o_z = cluster_coeffs_[i]->values[2];

			if (g.o_z < 0) {
				g.o_x *= -1;
				g.o_y *= -1;
				g.o_z *= -1;
			}

			face_geom_desc_.push_back(g);
			face_avg_bbox_lengths_ += g.bb_x + g.bb_y + g.bb_z;
		}
		face_avg_bbox_lengths_ /= cluster_coeffs_.size();

		for (int i = 0; i < cluster_indices_.size(); i++) {
			for (int j = i; j < cluster_indices_.size(); j++) {
				std::cout << i << "," << j << " : " << face_similarity(i, j) << "\n";
			}
		}
	}
  
  

  
  
  void extract_graph(std::vector<FaceCluster>& faces, std::vector<std::vector<int>>& adjlist, std::map<std::tuple<int,int>, FaceEdgeProps> edgeprops) {
    pcl::ExtractIndices<pcl::PointXYZ> ext;
    ext.setInputCloud(cloud_);
    ext.setNegative(false);
    
    for(int i=0; i<cluster_indices_->size(); i++) {
      FaceCluster fc;
      ext.setIndices(cluster_indices_[i]);
      ext.filter(fc.cloud);
      fc.geomdesc = face_geom_desc_[i];
      int res = computeCentroid(fc.cloud, fc.centroid);
      assert(res == fc.cloud.size());
      
      faces.push_back(fc);
    }
    
    adjlist = adjlist_;
    
    for(int i=0; i<adjlist_->size(); i++) {
      for(int j=0; j<adjlist_[i].size(); j++) {
        std::tuple<int,int> e = std::make_tuple(i, adjlist[i][j]);
        
        assert(e.first != e.second);
        if(e.first > e.second)
          continue;
        
        FaceEdgeProps prop;
        prop.centroid_relpos = faces[e.first].centroid - faces[e.second].centroid;
        edgeprops[e] = prop;
      }
    }
  }

  

  std::vector<pcl::PointIndices::Ptr> cluster_indices_;
  std::vector<pcl::ModelCoefficients::Ptr> cluster_coeffs_;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
  std::vector<std::vector<int>> adjlist_;
  std::vector<GeomDescriptors> face_geom_desc_;
  float face_avg_bbox_lengths_;

  typename pcl::search::KdTree<PointT>::Ptr kdtree_;
  
  typename pcl::PointCloud<PointT>::Ptr cloud_;
  double p_pc_dist_th_;
  double p_pc_samples_max_dist_;
  double p_pc_eps_angle_;
  int p_pc_min_points_;
  double p_norm_est_k_;
  double p_fc_maxrad_;
  double p_adj_sz_;
  int p_adj_k_;
  bool p_proj_;
};

*/





std::string g_inputfile;
double g_pc_dist_th = 0.05;
double g_pc_sample_max_dist = 1;
double g_pc_eps_angle = 0.05235987755;
double g_pc_min_points = 50;
double g_norm_est_k = 50;
double g_fc_maxrad = 0.05;
double g_adj_sz = 0.05;
int g_adj_k = 50;
bool g_project_points = false;
bool g_use_coolwarm_vis = false;
bool g_proj = false;

int
main (int argc, char** argv)
{
	if (argc == 1) {
		std::cout << "USAGE: " << argv[0] << " config-file\n";
		return 1;
	}

	std::map<std::string, std::string> param_map;

	if (parse_config_file(argv[1], param_map) == -1) {

		getchar();
		return 1;
	}

  FaceGraphParameters params;
	try_parse_param(param_map, "inputfile", g_inputfile);
	try_parse_param(param_map, "pc_dist_th", params.pc_dist_th);
	try_parse_param(param_map, "pc_sample_max_dist", params.pc_samples_max_dist);
	try_parse_param(param_map, "pc_eps_angle", params.pc_eps_angle);
	try_parse_param(param_map, "pc_min_points", params.pc_min_points);
	//try_parse_param(param_map, "norm_est_rad", params.norm_est_rad);
	try_parse_param(param_map, "norm_est_k", params.norm_est_k);
	try_parse_param(param_map, "fc_maxrad", params.fc_maxrad);
	try_parse_param(param_map, "adj_sz", params.adj_sz);
	try_parse_param(param_map, "adj_k", params.adj_k);
	try_parse_param(param_map, "project_points", g_project_points);
	try_parse_param(param_map, "use_coolwarm_vis", g_use_coolwarm_vis);
	try_parse_param(param_map, "proj", params.proj);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if(load_pcd_ply<pcl::PointXYZ>(g_inputfile, cloud) == -1)
    return 1;

  std::vector<FaceCluster<pcl::PointXYZ>> faces;
  std::vector<std::vector<int>> adjlist; 
  std::map<std::tuple<int, int>, FaceEdgeProps> edgeprops;

  build_face_graph(params, cloud, faces, adjlist, edgeprops);

  for (int i = 0; i < adjlist.size(); i++) {
	  std::cout << "Cluster " << i << ": ";
	  for (int j = 0; j < adjlist[i].size(); j++) {
		  std::cout << adjlist[i][j] << " ";
	  }
	  std::cout << "\n";
  }


  std::vector < std::tuple<float, int, int, int>> sim_list;

  for (int i = 0; i < faces.size(); i++) {

	  for (int j = i+1; j < faces.size(); j++) {	  
		  std::vector<int>& n_min = adjlist[i].size() <= adjlist[j].size() ? adjlist[i] : adjlist[j];
		  std::vector<int>& n_max = adjlist[i].size() > adjlist[j].size() ? adjlist[i] : adjlist[j];
		  float avgsim = 0;

		  if (n_min.empty()) {
			  continue;
		  }

		  for (int k = 0; k < n_min.size(); k++) {
			  float ms = 0;
			  for (int m = 0; m < n_max.size(); m++) {
				  float s = faces[n_min[k]].compute_similarity(faces[n_max[m]]);
				  if (ms < s)
					  ms = s;
			  }
			  avgsim += ms;

		  }
		  avgsim /= n_min.size();
		  std::cout << i << " " << j << ": " << avgsim << " " << n_max.size() - n_min.size() << "\n";

		  sim_list.push_back(std::make_tuple(avgsim, i, j, n_max.size() - n_min.size()));
		
	  }


	 
  }

  std::sort(sim_list.begin(), sim_list.end());

  for (auto it = sim_list.begin(); it != sim_list.end(); ++it) {
	  std::cout << std::get<1>(*it) << " " << std::get<2>(*it) << " : " << std::get<0>(*it) << " " << std::get<3>(*it) << "\n";
  }

  //visualize_clusters<pcl::PointXYZ>(cloud, seg.get_clusters(), g_use_coolwarm_vis ? COLOR_MAP_COOLWARM : COLOR_MAP_RAINBOW, 1);
  
  return (0);
}


