#include <algorithm>
#include <iostream>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include <cassert>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include "common.h"
#include "parse.h"
#include "graph.h"
#include "face_segmentation.hpp"




//Geometric descriptors for each face cluster.
struct FaceGeomDescriptors {
  float bb_x, bb_y, bb_z;
  float o_x, o_y, o_z;
  int num_points;
  float bb_edge_sum;
};


//All properties for each node/cluster.
struct FaceClusterProps {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  FaceGeomDescriptors gd;
  pcl::PointXYZ centroid, bbox_centre;
  
  std::string to_string() {
    std::stringstream ss;
    ss << "centroid = " << centroid << "\n";
    ss << "bbox_centre = " << bbox_centre << "\n";
    ss << "bb = " << gd.bb_x << " " <<  gd.bb_y << " " << gd.bb_z << "\n";
    ss << "o = " << gd.o_x << " " <<  gd.o_y << " " << gd.o_z << "\n";
    ss << "bb_edge_sum = " << gd.bb_edge_sum << "\n";
    ss << "num_points = " << gd.num_points << "\n";
    return ss.str();
  }
};




typedef UndirectedMapGraph<FaceClusterProps, int> FaceGraph;










//Take a point cloud and face segmentation parameters as input. Cluster it into faces, return the initialised graph. 
FaceGraph create_face_graph(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, FaceSegmentorParameters const& params) {
  
  FaceGraph graph;
  
  FaceSegmentor<pcl::PointXYZ> seg(cloud, params);
  seg.run();
  
  
  std::map<int,int> cluster_graph_ids; //Map FaceSegmentor cluster indices to node IDs, just for completeness.
  
  //For each cluster, initialise a FaceClusterProps struct for it and add it to the graph.
  for(int i=0; i<seg.get_clusters_indices().size(); i++) {
    FaceClusterProps fc;
    
    fc.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    seg.extract_cluster(i, fc.cloud); //Extract points from input point cloud into new cloud.
    
    //Compute min/max coordinates for points in cluster.
    float bb_x_min, bb_x_max, bb_y_min, bb_y_max, bb_z_min, bb_z_max;
    bb_x_min = bb_y_min = bb_z_min = std::numeric_limits<float>::max();
    bb_x_max = bb_y_max = bb_z_max = std::numeric_limits<float>::lowest();
    
    
    for(int p=0; p<fc.cloud->points.size(); p++) {
      auto s = fc.cloud->points[p];
      if(s.x < bb_x_min) bb_x_min = s.x;
      if(s.y < bb_y_min) bb_y_min = s.y;
      if(s.z < bb_z_min) bb_z_min = s.z;
      if(s.x > bb_x_max) bb_x_max = s.x;
      if(s.y > bb_y_max) bb_y_max = s.y;
      if(s.z > bb_z_max) bb_z_max = s.z;
    }
    
    fc.gd.bb_x = std::abs(bb_x_max - bb_x_min);
    fc.gd.bb_y = std::abs(bb_y_max - bb_y_min);
    fc.gd.bb_z = std::abs(bb_z_max - bb_z_min);
    fc.gd.bb_edge_sum = fc.gd.bb_x + fc.gd.bb_y + fc.gd.bb_z;
    
    fc.bbox_centre.x = (bb_x_max + bb_x_min) / 2;
    fc.bbox_centre.y = (bb_y_max + bb_y_min) / 2;
    fc.bbox_centre.z = (bb_z_max + bb_z_min) / 2;
    

    //Normal of fitted plane for cluster.
    fc.gd.o_x = seg.get_cluster_coeffs()[i]->values[0];
    fc.gd.o_y = seg.get_cluster_coeffs()[i]->values[1];
    fc.gd.o_z = seg.get_cluster_coeffs()[i]->values[2];
    
    if(fc.gd.o_z < 0) {
      fc.gd.o_x *= -1.0f;
      fc.gd.o_y *= -1.0f;
      fc.gd.o_z *= -1.0f;
    }
    
    fc.gd.num_points = fc.cloud->points.size();

    computeCentroid(*(fc.cloud), fc.centroid);
    
    int n = graph.add_node();
    graph.get_node_props(n) = fc;
    cluster_graph_ids[i] = n;
  }
  
  
  std::vector<std::vector<int>> const& adjlist = seg.get_adjlist();
  
  //Check for no self loops. This should never happen.
  for(int i=0; i<adjlist.size(); i++) {
    for(int j=0; j<adjlist[i].size(); j++) {
      if(adjlist[i][j] == i) {
        throw std::runtime_error("Self loop found");
      }
    }
  }
  
  //Add edge information to graph.
  for(int i=0; i<adjlist.size(); i++) {
    for(int j=0; j<adjlist[i].size(); j++) {
      graph.add_edge(cluster_graph_ids.at(i), cluster_graph_ids.at(adjlist[i][j]), true);
    }
  }
  
  return graph;
}







//Returns value in [0,1] describing similarity of two nodes/clusters. 1 means they are identical.
//ai and bi are IDs of nodes in graph.
float compute_face_similarity(int ai, int bi, FaceGraph& graph) {
    FaceClusterProps a = graph.get_node_props(ai);
    FaceClusterProps b = graph.get_node_props(bi);

	  float bbox_diff = abs(a.gd.bb_x - b.gd.bb_x) +
		  abs(a.gd.bb_y - b.gd.bb_y) +
		  abs(a.gd.bb_z - b.gd.bb_z);

	  bbox_diff /= std::min(a.gd.bb_edge_sum, b.gd.bb_edge_sum) * 10;
	  //0 = perfect match
	  //1 = difference is one tenth of avg bbox edge sum lengths.

	  float dp1 = a.gd.o_x * b.gd.o_x +
		  a.gd.o_y * b.gd.o_y +
		  a.gd.o_z * b.gd.o_z;

	  float dp2 = (-a.gd.o_x) * b.gd.o_x +
		  (-a.gd.o_y) * b.gd.o_y +
		  (-a.gd.o_z) * b.gd.o_z;
    
    float dp = std::min(std::abs(dp1), std::abs(dp2));
    
	  if (dp > 1) dp = 1;

	  float o_diff = acos(dp) / (15.0f / 180.0f * M_PI);
	  //0 = perfect match
	  //1 = difference in orientation is 15 degrees


    float sz_min = std::min(a.cloud->size(), b.cloud->size());
    float sz_max = std::max(a.cloud->size(), b.cloud->size());

    float sz_diff = (sz_max - sz_min) / sz_min * 10;
    

	  if (bbox_diff > 1) {
		  bbox_diff = 1;
	  }
	  if (o_diff > 1) {
		  o_diff = 1;
	  }
    if(sz_diff > 1) {
      sz_diff = 1;
    }

	  return 1 - (bbox_diff + o_diff + sz_diff) / 3.0f;
	}






//Find most similar pairs of neighbours for clusters a and b and return their average similarity.
//The number of pairs is equal to the size of the smaller neighbourhood.
//Each pair has one neighbour of a and one of b. A cluster may only appear once for each neighbourhood.
float compute_neighbourhood_similarity(int ai, int bi, FaceGraph& graph) {
  std::set<int> const& n_min_set = graph.adjacent(ai).size() <= graph.adjacent(bi).size() ? graph.adjacent(ai) : graph.adjacent(bi);
  std::set<int> const& n_max_set = graph.adjacent(ai).size() > graph.adjacent(bi).size() ? graph.adjacent(ai) : graph.adjacent(bi);

  std::vector<int> n_min(n_min_set.begin(), n_min_set.end());
  std::vector<int> n_max(n_max_set.begin(), n_max_set.end());

  //Return 0 if smaller neighbourhood is empty or size difference between neighbourhoods > 2
  if (n_min.empty() || n_max.size() - n_min.size() > 2) {
    return 0;
  }
  
  //Over all possible correspondences between neighbourhoods, find one with greatest sum of face-similarity values.
  float max_sim = 0;
  
  //Iterate through every k combination of {0,1,...,n_max.size()}, where k = n_min.size().
  //Compute sum of face_sim(n_min[i], m_max[max_try_idx[i]]), for i=0,...,n_min.size()-1
   
  std::vector<int> max_try_idx(n_min.size());
  //std::vector<int> best_idx;
  
  //Initialise max_try_idx to (0,1,...,n_max.size()-1)
  for(int i=0; i<max_try_idx.size(); i++) {
    max_try_idx[i] = i;
  }
  
  //Iterate through all k combinations.
  do {
    float s = 0;
    for(int i=0; i<n_min.size(); i++) {
      s += compute_face_similarity(n_min[i], n_max[max_try_idx[i]], graph);
    }
    
    if(max_sim < s) {
      max_sim = s;
      //best_idx = max_try_idx;
    }
    std::reverse(max_try_idx.begin()+n_min.size(), max_try_idx.end());
  } while(std::next_permutation(max_try_idx.begin(), max_try_idx.end()));
  
  return max_sim / n_min.size();
}



/*
TRY_REPLACE_NODE(t)
  Fs = set of nodes that are face-similar to t          (face-similarity >= p_gr_fs_sim)
  Ns = set of nodes that are neighbourhood-similar to t (neighbourhood-similarity >= p_gr_ns_sim)
  
  if( Fs.size() > p_gr_fs_max_sz ) return;
  
  if( Ns.size() < p_gr_ns_min_sz ) return;
  
  if any two nodes in Ns have face-similarity < p_gr_ns_inter_face_sim:
    return
  
  if average face-similarity between t and nodes in Ns > p_gr_ns_max_tgt_sim_th:
    return
  
  r = node in Ns with greatest neighbourhood similarity to t.
  
  replace points of t with points of r.
  
*/


//This function attempts the above operation on every node, and returns false if no applicable node exists.
bool do_bad_node_replacement( FaceGraph& graph,
                              float p_gr_fs_sim,
                              float p_gr_ns_sim,
                              int p_gr_fs_max_sz,
                              int p_gr_ns_min_sz,
                              float p_gr_ns_inter_face_sim,
                              float p_gr_ns_max_avg_tgt_sim) {
  
  std::vector<int> nodes(graph.nodes().begin(), graph.nodes().end());

  for(auto it=nodes.begin(); it!=nodes.end(); ++it) {
    std::vector<int> Ns, Fs;
    
    for(auto it2=nodes.begin(); it2!=nodes.end(); ++it2) {
      if(*it == *it2)
        continue;
    
      if(compute_face_similarity(*it, *it2, graph) >= p_gr_fs_sim)
        Fs.push_back(*it2);
      
      if(compute_neighbourhood_similarity(*it, *it2, graph) >= p_gr_ns_sim)
        Ns.push_back(*it2);
    }
    
    if(Fs.size() > p_gr_fs_max_sz)
      continue;
    
    if(Ns.size() < p_gr_ns_min_sz)
      continue;
    
    
    bool ns_coherent = true;
    
    for(auto ns_it1=Ns.begin(); ns_it1!=Ns.end(); ++ns_it1) {
      for(auto ns_it2=Ns.begin(); ns_it2!=Ns.end(); ++ns_it2) {
        if(*ns_it1 == *ns_it2)
          continue;
        
        if(compute_face_similarity(*ns_it1, *ns_it2, graph) < p_gr_ns_inter_face_sim) {
          ns_coherent = false;
          break;
        }
      }
      if(!ns_coherent)
        break;
    }
    
    if(!ns_coherent)
      continue;
    
    
    float avg_ns_tgt_sim = 0;
    
    for(auto ns_it=Ns.begin(); ns_it!=Ns.end(); ++ns_it)
      avg_ns_tgt_sim += compute_face_similarity(*it, *ns_it, graph);
    
    avg_ns_tgt_sim /= Ns.size();
    
    if(avg_ns_tgt_sim > p_gr_ns_max_avg_tgt_sim)
      continue;
    
    int best_replacement_node = -1;
    float best_replacement_sim  = -1;
    
    for(auto ns_it=Ns.begin(); ns_it!=Ns.end(); ++ns_it) {
      float s = compute_neighbourhood_similarity(*it, *ns_it, graph);
      if(s > best_replacement_sim) {
        best_replacement_node = *ns_it;
        best_replacement_sim = s;
      }
    }
    
    std::cout << "Replace " << *it << " with " << best_replacement_node << "\n";
    

    graph.get_node_props(*it).cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PointXYZ src_bbox_centre = graph.get_node_props(best_replacement_node).bbox_centre;
    pcl::PointXYZ dst_bbox_centre = graph.get_node_props(*it).bbox_centre;
    
    Eigen::Affine3f transf = Eigen::Affine3f::Identity();
    transf.translation() << dst_bbox_centre.x - src_bbox_centre.x, dst_bbox_centre.y - src_bbox_centre.y, dst_bbox_centre.z - src_bbox_centre.z;
    
    std::cout << "Source centre: " << src_bbox_centre << "\n";
    std::cout << "Dest centre: " << dst_bbox_centre << "\n";
    std::cout << "Transform: " << transf.translation() << "\n";
    
    pcl::transformPointCloud(*(graph.get_node_props(best_replacement_node).cloud), *(graph.get_node_props(*it).cloud), transf);
    graph.get_node_props(*it).gd = graph.get_node_props(best_replacement_node).gd;

    return true;
  }

  return false;
}



//Reconstruct model from graph.
//cloud is an empty PointCloud, graph cluster are copied into it.
//cluster_indices is set to indices of each cluster in cloud.
void reconstruct_graph( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                        std::vector<pcl::PointIndices::Ptr>& cluster_indices,
                        FaceGraph& graph) {
  
  cloud->clear();
  cluster_indices.clear();
  
  for(auto it=graph.nodes().begin(); it != graph.nodes().end(); ++it) {
    pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
    
    for(int j=cloud->size(); j < cloud->size() + graph.get_node_props(*it).cloud->size(); j++) {
      indices->indices.push_back(j);
    }
    
    cluster_indices.push_back(indices);
    *cloud += *(graph.get_node_props(*it).cloud);
    
  }
  
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;
}





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
float p_gr_fs_sim = 0.8;
float p_gr_ns_sim = 0.8;
int p_gr_fs_max_sz = 1;
int p_gr_ns_min_sz = 2;
float p_gr_ns_inter_face_sim = 0.8;
float p_gr_ns_max_avg_tgt_sim = 0.8;
  
int
main (int argc, char** argv)
{
	if (argc == 1) {
		std::cout << "USAGE: " << argv[0] << " config-file\n";
		return 1;
	}

	std::map<std::string, std::string> param_map;

	if (parse_config_file(argv[1], param_map) == -1) {
		return 1;
	}

  FaceSegmentorParameters params;
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
  
	try_parse_param(param_map, "gr_fs_sim", p_gr_fs_sim);
	try_parse_param(param_map, "gr_ns_sim", p_gr_ns_sim);
	try_parse_param(param_map, "gr_fs_max_sz", p_gr_fs_max_sz);
	try_parse_param(param_map, "gr_ns_min_sz", p_gr_ns_min_sz);
	try_parse_param(param_map, "gr_ns_inter_face_sim", p_gr_ns_inter_face_sim);
	try_parse_param(param_map, "gr_ns_max_avg_tgt_sim", p_gr_ns_max_avg_tgt_sim);


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if(load_pcd_ply<pcl::PointXYZ>(g_inputfile, cloud) == -1)
    return 1;
    
  
  FaceGraph graph = create_face_graph(cloud, params);

  std::string adjliststr = graph.adjlist_str();
  
  
  
  {
    ofstream out("clusters.txt");
    if(!out) {
      std::cout << "Could not open clusters.txt\n";
      return 2;
    }
    
    for(auto it=graph.nodes().begin(); it!=graph.nodes().end(); ++it) {
      out << *it << ": \n";
      out << graph.get_node_props(*it).to_string() << "\n";
    }
    
    out << "\n\n" << adjliststr << "\n\n";
    
    for(auto it1=graph.nodes().begin(); it1!=graph.nodes().end(); ++it1) {
      for(auto it2=graph.nodes().begin(); it2!=graph.nodes().end(); ++it2) {
        out << *it1 << " " << *it2 << ": " << compute_face_similarity(*it1, *it2, graph) << " " << compute_neighbourhood_similarity(*it1, *it2, graph) << "\n";
      }
    }
  }
  
  //Repeatly apply graph operation while some node is applicable.
  while(do_bad_node_replacement(graph, p_gr_fs_sim, p_gr_ns_sim, p_gr_fs_max_sz, p_gr_ns_min_sz, p_gr_ns_inter_face_sim, p_gr_ns_max_avg_tgt_sim)) {}

  pcl::PointCloud<pcl::PointXYZ>::Ptr repairedcloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointIndices::Ptr> cluster_indices;
  
  reconstruct_graph(repairedcloud, cluster_indices, graph);
  
  visualize_clusters<pcl::PointXYZ>(repairedcloud, cluster_indices, g_use_coolwarm_vis ? COLOR_MAP_COOLWARM : COLOR_MAP_RAINBOW, 12345);

  //~ pcl::visualization::PCLVisualizer viewer(std::string("PCLVisualizer"));
  //~ viewer.addPointCloud<pcl::PointXYZ>(repairedcloud);
  //~ //viewer.addPointCloud<pcl::PointXYZ>(repairedcloud, "cloud2", 1);
  
  //~ while (!viewer.wasStopped()) {
    //~ viewer.spinOnce (100);
    //~ boost::this_thread::sleep (boost::posix_time::microseconds (100000));
   //~ }
   


  return (0);
}


