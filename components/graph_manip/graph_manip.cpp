#include <algorithm>
#include <iostream>
#include <fstream>
#include <limits>
#include <map>
#include <set>
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





struct FaceGeomDescriptors {
  float bb_x, bb_y, bb_z;
  float o_x, o_y, o_z;
  int num_points;
  float bb_edge_sum;  
};

struct FaceClusterProps {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  FaceGeomDescriptors gd;
  pcl::PointXYZ centroid, bbox_centre;
};




typedef UndirectedMapGraph<FaceClusterProps, int> FaceGraph;











FaceGraph create_face_graph(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, FaceSegmentorParameters const& params) {
  
  FaceGraph graph;
  
  FaceSegmentor<pcl::PointXYZ> seg(cloud, params);
  seg.run();
  
  
  std::map<int,int> cluster_graph_ids;
  
  for(int i=0; i<seg.get_clusters_indices().size(); i++) {
    FaceClusterProps fc;
    
    fc.cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    seg.extract_cluster(i, fc.cloud);
    
    float bb_x_min, bb_x_max, bb_y_min, bb_y_max, bb_z_min, bb_z_max;
    bb_x_min = bb_y_min = bb_z_min = std::numeric_limits<float>::max();
    bb_x_max = bb_y_max = bb_z_max = std::numeric_limits<float>::min();
    
    for(int p=0; p<fc.cloud->points.size(); p++) {
      bb_x_min = std::min(bb_x_min, fc.cloud->points[p].x);
      bb_y_min = std::min(bb_y_min, fc.cloud->points[p].y);
      bb_z_min = std::min(bb_z_min, fc.cloud->points[p].z);
      bb_x_max = std::max(bb_x_max, fc.cloud->points[p].x);
      bb_y_max = std::max(bb_y_max, fc.cloud->points[p].y);
      bb_z_max = std::max(bb_z_max, fc.cloud->points[p].z);
    }
    
    fc.gd.bb_x = bb_x_max - bb_x_min;
    fc.gd.bb_y = bb_y_max - bb_y_min;
    fc.gd.bb_z = bb_z_max - bb_z_min;
    fc.gd.bb_edge_sum = fc.gd.bb_x + fc.gd.bb_y + fc.gd.bb_z;
    
    fc.bbox_centre.x = bb_x_min + fc.gd.bb_x / 2;
    fc.bbox_centre.y = bb_y_min + fc.gd.bb_y / 2;
    fc.bbox_centre.z = bb_z_min + fc.gd.bb_z / 2;
    
    
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
  
  for(int i=0; i<adjlist.size(); i++) {
    for(int j=0; j<adjlist[i].size(); j++) {
      graph.add_edge(cluster_graph_ids.at(i), cluster_graph_ids.at(adjlist[i][j]), true);
    }
  }
  
  return graph;
}








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



bool set_next_perm(std::vector<int>& p, int mx) {
  for(int i=p.size()-1; i>=0; i--) {
    if(p[i] == mx) {
      p[i] = 0;
      if(i == 0)
        return true;
    } else {
      p[i]++;
      break;
    }
  }
  return false;
}


//Find most similar pairs of neighbours for clusters a and b and return their average similarity.
//The number of pairs is equal to the size of the smaller neighbourhood.
//Each pair has one neighbour of a and one of b. A cluster may only appear once for each neighbourhood.
float compute_neighbourhood_similarity(int ai, int bi, FaceGraph& graph) {
  std::set<int> const& n_min_set = graph.adjacent(ai).size() <= graph.adjacent(bi).size() ? graph.adjacent(ai) : graph.adjacent(bi);
  std::set<int> const& n_max_set = graph.adjacent(ai).size() > graph.adjacent(bi).size() ? graph.adjacent(ai) : graph.adjacent(bi);

  std::vector<int> n_min(n_min_set.begin(), n_min_set.end());
  std::vector<int> n_max(n_max_set.begin(), n_max_set.end());

  if (n_min.empty() || n_max.size() - n_min.size() > 2) {
    return 0;
  }
  
  float max_sim = 0;
  
  std::vector<int> max_try_idx(n_min.size());
  //std::vector<int> best_idx;
  
  do {
    float s = 0;
    for(int i=0; i<n_min.size(); i++) {
      s += compute_face_similarity(n_min[i], n_max[max_try_idx[i]], graph);
    }
    
    if(max_sim < s) {
      max_sim = s;
      //best_idx = max_try_idx;
    }
    
  } while(!set_next_perm(max_try_idx, n_max.size()-1));
  
  return max_sim / n_min.size();
}



/*
TRY_REPLACE_NODE(t)
  Ns = set of other nodes with similar neighbourhoods (similar means neighbourhood similarity >= p_gr_sim_th_nghbr)
  
  if Ns.size() < p_gr_min_num_sim_nghbrs:
    return
  
  BN = subset of Ns, where each node is dissimilar to some other node in Ns. (disimilar means face similarity < p_gr_inter_nghbr_sim_th)
          Or, is too face-similar to t (face-similarity >= _p_gr_inter_nghbr_tgt_sim_th)
  
  if BN.size() / Ns.size() > p_gr_inter_nghbr_bad_tol:
    return
  
  //We have confirmed Ns is a reoccurring face+neighbourhood structure. Now insure t is face-dissimilar to all nodes in Ns.
  
  ReplacementNode = the node r in Ns, that's not in BN, that has the best neighbourhood similarity to t.
  
  t.points = r.points
*/


//This function essentially runs the above algorithm on an arbitrary node and returns true, else returns false if no nodes meets all the criteria.
bool do_bad_node_replacement( FaceGraph& graph,
                              float p_gr_sim_th_nghbr,          //Minimum neighbourhood similarity needed between two nodes to be considered as having similar neighbourhoods. 
                              float p_gr_min_num_sim_nghbrs,    //Minimum number of similar neighbourhoods found needed to proceeded to replacement.
                              float p_gr_inter_nghbr_sim_th,    //Minimum similarity between any two nodes in group of similar neighbourhoods needed to consider those nodes as part of a coherent group.
                              float p_gr_inter_nghbr_tgt_sim_th,//Minimum face-similarity between tgt node and a neighbour-similar one to be considered similar.
                              float p_gr_inter_nghbr_bad_tol) { //Proportion of noncoherent neighbourhoods allowed.
  
  
  std::vector<int> nodes(graph.nodes().begin(), graph.nodes().end());

  int target_node = -1;
  int replacement_node = 0;
  float replacement_sim = 0;
   
   
  for(int i=0; i<nodes.size(); i++) {
    std::cout << "\nCluster " << nodes[i] << " at " << graph.get_node_props(nodes[i]).bbox_centre << "\n";
    
    std::vector<std::tuple<float, int>> nodes_with_similar_neighbours;
    
    for(int j=0; j<nodes.size(); j++) {
      if(i == j)
        continue;
      
      float s = compute_neighbourhood_similarity(nodes[i], nodes[j], graph);
      
      if(s > p_gr_sim_th_nghbr)
        nodes_with_similar_neighbours.push_back(std::make_tuple(s, nodes[j]));
    }
  
  std::cout << "Found " << nodes_with_similar_neighbours.size() << " neighbour-similar nodes.\n";
  
  
    
   if(nodes_with_similar_neighbours.size() < p_gr_min_num_sim_nghbrs) {
     std::cout << "Not enough neighbour-similar nodes.\n";
    continue;
  }
    
   std::sort(nodes_with_similar_neighbours.begin(), nodes_with_similar_neighbours.end(), std::greater<std::tuple<float,int>>());
  
   std::set<int> bad_neighbours;
   for(int j=0; j<nodes_with_similar_neighbours.size(); j++) {
     for(int k=j+1; k<nodes_with_similar_neighbours.size(); k++) {
       if(compute_face_similarity(
            std::get<1>(nodes_with_similar_neighbours[j]), 
            std::get<1>(nodes_with_similar_neighbours[k]), 
            graph) < p_gr_inter_nghbr_sim_th) {
            
         bad_neighbours.insert(std::get<1>(nodes_with_similar_neighbours[j]));
         bad_neighbours.insert(std::get<1>(nodes_with_similar_neighbours[k]));
       }
     }
     
     if(compute_face_similarity(nodes[i], std::get<1>(nodes_with_similar_neighbours[j]), graph) >= p_gr_inter_nghbr_tgt_sim_th)
      bad_neighbours.insert(std::get<1>(nodes_with_similar_neighbours[j]));
   }
   
   std::cout << "Found " << bad_neighbours.size() << " incoherent neighbour-similar nodes.\n";
   
   if(((float)bad_neighbours.size()) / nodes_with_similar_neighbours.size() > p_gr_inter_nghbr_bad_tol) {
    std::cout << "Too many incoherent neighbour-similar nodes.\n";
    continue;
   }
   
   
   for(int j=0; j<nodes_with_similar_neighbours.size(); j++) {
     if(std::get<0>(nodes_with_similar_neighbours[j]) > replacement_sim && bad_neighbours.count(std::get<1>(nodes_with_similar_neighbours[j])) == 0) {
       replacement_node = std::get<1>(nodes_with_similar_neighbours[j]);
       replacement_sim = std::get<0>(nodes_with_similar_neighbours[j]);
     }
     
   }
   
   target_node = nodes[i];
   break;
  }
  
  if(target_node == -1)
    return false;
  
  
  std::cout << "Replace node " << target_node << " with node " << replacement_node << "\n";
   
  pcl::PointXYZ src_bbox_centre, dst_bbox_centre, old_centroid;
  src_bbox_centre = graph.get_node_props(replacement_node).bbox_centre;
  dst_bbox_centre = graph.get_node_props(target_node).bbox_centre;
  old_centroid = graph.get_node_props(target_node).centroid;
  
  graph.get_node_props(target_node) = graph.get_node_props(replacement_node);
  graph.get_node_props(target_node).cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  
  Eigen::Affine3f transf = Eigen::Affine3f::Identity();
  
  transf.translation() << dst_bbox_centre.x - src_bbox_centre.x, dst_bbox_centre.y - src_bbox_centre.y, dst_bbox_centre.z - src_bbox_centre.z; 
  
  pcl::transformPointCloud(*(graph.get_node_props(replacement_node).cloud), *(graph.get_node_props(target_node).cloud), transf);
  
  graph.get_node_props(target_node).bbox_centre = dst_bbox_centre;
  graph.get_node_props(target_node).centroid = old_centroid;
  
  return true;  
}



//Reconstruct model from graph
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
float p_gr_sim_th_nghbr = 0.8;
float p_gr_min_num_sim_nghbrs = 2;
float p_gr_inter_nghbr_sim_th = 0.7;
float p_gr_inter_nghbr_tgt_sim_th = 0.7;
float p_gr_inter_nghbr_bad_tol = 0.2;
  
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
	try_parse_param(param_map, "proj", params.proj);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  if(load_pcd_ply<pcl::PointXYZ>(g_inputfile, cloud) == -1)
    return 1;
    
  
  FaceGraph graph = create_face_graph(cloud, params);

  std::string adjliststr = graph.adjlist_str();
  
  {
    ofstream out("graphinfo.txt");
    if(!out)
      std::cout << "Could not open adjlist.txt\n";
    else {
      out << adjliststr << "\n\n";
      
      for(auto it=graph.nodes().begin(); it!=graph.nodes().end(); ++it) {
        pcl::PointXYZ bbox_centre = graph.get_node_props(*it).bbox_centre;
        pcl::PointXYZ centroid = graph.get_node_props(*it).centroid;
        
        out << *it << ": " << bbox_centre << "  " << centroid << "\n";
      }
    }
  }
  
  while(do_bad_node_replacement(graph, p_gr_sim_th_nghbr, p_gr_min_num_sim_nghbrs, p_gr_inter_nghbr_sim_th, p_gr_inter_nghbr_tgt_sim_th, p_gr_inter_nghbr_bad_tol)) {}


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


