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
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "common.h"

#pragma once


struct GeomDescriptors {
  double bb_x, bb_y, bb_z;
  double o_x, o_y, o_z;
  float bbox_edge_sum;
};

template<typename PointT>
struct FaceCluster {
  typename pcl::PointCloud<PointT>::Ptr cloud;
  GeomDescriptors geomdesc;
  PointT centroid, bbox_centre;

	float compute_similarity(FaceCluster const& n) {
  float bbox_diff = abs(geomdesc.bb_x - n.geomdesc.bb_x) +
	  abs(geomdesc.bb_y - n.geomdesc.bb_y) +
	  abs(geomdesc.bb_z - n.geomdesc.bb_z);

  bbox_diff /= abs((geomdesc.bbox_edge_sum < n.geomdesc.bbox_edge_sum ? geomdesc.bbox_edge_sum : n.geomdesc.bbox_edge_sum) * 10);

  //0 = perfect match
  //1 = difference is one tenth of avg bbox edge sum lengths.
  float dp = geomdesc.o_x * n.geomdesc.o_x +
	  geomdesc.o_y * n.geomdesc.o_y +
	  geomdesc.o_z * n.geomdesc.o_z;

  if (dp > 1) dp = 1;

  float o_diff = acos(dp) / (15.0f / 180.0f * M_PI);
  //0 = perfect match
  //1 = difference in orientation is 15 degrees
  if (bbox_diff > 1) {
	  bbox_diff = 1;
  }
  //if (bbox_diff < 0) {
	 // bbox_diff = 0;
  //}
  if (o_diff > 1) {
	  o_diff = 1;
  }
  // density descriptor
  float density_d = 0;
  int self_density = cloud->points.size();
  int n_density = n.cloud->points.size();
  int density_diff = abs(self_density - n_density);
  

  int bigger_cloud_size = self_density;
  if (bigger_cloud_size < n_density) {
	  bigger_cloud_size = n_density;
  }
  density_d = (float)density_diff / (float)bigger_cloud_size;

  //if(density_d > 1) {
//std::cout << "density_d: " << density_d << " bbox_diff: " << bbox_diff << " big size: " << bigger_cloud_size << "\n";
  //}

  return 1 - (bbox_diff *0.5f) -( density_d * 0.5f);
}
};

struct FaceEdgeProps {
  pcl::PointXYZ centroid_relpos;
};


struct FaceSegmentorParameters {
  double norm_est_k;
  double pc_dist_th;
  double pc_samples_max_dist; 
  double pc_eps_angle; 
  int pc_min_points;
  double fc_maxrad;
  double adj_sz;
  int adj_k;
  bool proj;
};

template<typename PointT>
class FaceSegmentor {
  public:
  

  
  
  FaceSegmentor( typename pcl::PointCloud<PointT>::Ptr cloud, 
                 FaceSegmentorParameters const& params) :
    cloud_normals_(new pcl::PointCloud<pcl::Normal>),
    kdtree_(new typename pcl::search::KdTree<PointT>),
    cloud_(cloud),
    p_norm_est_k_(params.norm_est_k),
    p_pc_dist_th_(params.pc_dist_th),
    p_pc_samples_max_dist_(params.pc_samples_max_dist),
    p_pc_eps_angle_(params.pc_eps_angle),
    p_pc_min_points_(params.pc_min_points),
    p_fc_maxrad_(params.fc_maxrad),
    p_adj_sz_(params.adj_sz),
    p_adj_k_(params.adj_k),
    p_proj_(params.proj)
  {
  }
  

   
  int run() {
    compute_normals();
    segment_planes();
    segment_faces();
    compute_adjacency();

    if (p_proj_)
      project_points();
    
    //compute_geom_descriptors();
    return 0;
  }
  
  std::vector<pcl::PointIndices::Ptr> const& get_clusters_indices() {
    return cluster_indices_;
  }
  
  std::vector<pcl::ModelCoefficients::Ptr> const& get_cluster_coeffs() {
    return cluster_coeffs_;
  }
  
  std::vector<std::vector<int>> const& get_adjlist() {
    return adjlist_;
  }
  
  void extract_cluster(int c, typename pcl::PointCloud<PointT>::Ptr pc) {
    if(c >= cluster_indices_.size())
      throw std::runtime_error("Invalid cluster id");
      
    pcl::ExtractIndices<pcl::PointXYZ> ext;
	  ext.setInputCloud(cloud_);
	  ext.setNegative(false);
    ext.setIndices(cluster_indices_[c]);
		ext.filter(*pc);
  }
    
    
  /*
  void extract_graph(std::vector<FaceCluster<PointT>>& faces, std::vector<std::vector<int>>& adjlist) {
	  pcl::ExtractIndices<pcl::PointXYZ> ext;
	  ext.setInputCloud(cloud_);
	  ext.setNegative(false);

	  for (int i = 0; i<cluster_indices_.size(); i++) {
		  FaceCluster<PointT> fc;
		  fc.cloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
		  ext.setIndices(cluster_indices_[i]);
		  ext.filter(*(fc.cloud));
		  fc.geomdesc = face_geom_desc_[i];
		  int res = computeCentroid(*(fc.cloud), fc.centroid);
		  assert(res == fc.cloud->size());

		  faces.push_back(fc);
	  }

	  adjlist = adjlist_;

	  for (int i = 0; i<adjlist_.size(); i++) {
		  for (int j = 0; j<adjlist_[i].size(); j++) {
			  std::tuple<int, int> e = std::make_tuple(i, adjlist[i][j]);

			  assert(std::get<0>(e) != std::get<1>(e));
			  if (std::get<0>(e) > std::get<1>(e))
				  continue;

			  FaceEdgeProps prop;
			  prop.centroid_relpos.x = faces[std::get<1>(e)].centroid.x - faces[std::get<0>(e)].centroid.x;
			  prop.centroid_relpos.y = faces[std::get<1>(e)].centroid.y - faces[std::get<0>(e)].centroid.y;
			  prop.centroid_relpos.z = faces[std::get<1>(e)].centroid.z - faces[std::get<0>(e)].centroid.z;
			  edgeprops[e] = prop;
		  }
	  }
  }

  */
  
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
      
      //std::cout << "Valid indices: " << valid_indices->indices.size() << "\n";
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
		//std::cout << "Cluster " << cl << "\n";
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
    
    
    //For each region...
    for(auto regit=region_counts.begin(); regit!=region_counts.end(); ++regit) {
      
      int x = std::get<0>(regit->first);
      int y = std::get<1>(regit->first);
      int z = std::get<2>(regit->first);
      
      //Count the points in the 27 region area: super_region_counts[cluster] -> n
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
      
      //it is an iterator for the map of counts for region <x,y,z>
      for(auto it=regit->second.begin(); it!=regit->second.end(); ++it) {
        //it2 is an iterator for the map of counts for the super region
        for(auto it2=super_region_counts.begin(); it2!=super_region_counts.end(); ++it2) {
          
          //A cluster can't be adjacent to itself.
          if(it->first == it2->first)
            continue;
          
          //t = (some cluster in region, some other cluster in super region)
          std::tuple<int,int> t = std::make_tuple(it->first, it2->first);
          
          //n is min number of points from either cluster.
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
    
    //for(int i=0; i<adjlist_.size(); i++) {
    //  std::cout << "Cluster " << i << ": ";
    //  for(int j=0; j<adjlist_[i].size(); j++) {
    //    std::cout << adjlist_[i][j] << " ";
      //}
     // std::cout << "\n";
    //}
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
  
  
  /*
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

			g.bbox_edge_sum = g.bb_x + g.bb_y + g.bb_z;

			face_geom_desc_.push_back(g);
			face_avg_bbox_lengths_ += g.bb_x + g.bb_y + g.bb_z;
		}
		face_avg_bbox_lengths_ /= cluster_coeffs_.size();

		//for (int i = 0; i < cluster_indices_.size(); i++) {
		//	for (int j = i; j < cluster_indices_.size(); j++) {
		//		std::cout << i << "," << j << " : " << face_similarity(i, j) << "\n";
		//	}
		//}
	}
  */
  

  


  std::vector<pcl::PointIndices::Ptr> cluster_indices_;
  std::vector<pcl::ModelCoefficients::Ptr> cluster_coeffs_;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
  std::vector<std::vector<int>> adjlist_;
  //std::vector<GeomDescriptors> face_geom_desc_;
  //float face_avg_bbox_lengths_;

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




//~ template<typename PointT>
//~ int build_face_graph(FaceGraphParameters const& params, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<FaceCluster<PointT>>& faces, std::vector<std::vector<int>>& adjlist, std::map<std::tuple<int,int>, FaceEdgeProps>& edgeprops) {
  
  //~ FaceGraphSegmentor<PointT> seg (cloud, params);
  //~ seg.run();
  //~ seg.extract_graph(faces, adjlist, edgeprops);
  //~ return 0;
//~ }
