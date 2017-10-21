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
#include <pcl/common.h>
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
      bb_x_max = std::min(bb_x_max, fc.cloud->points[p].x);
      bb_y_max = std::min(bb_y_max, fc.cloud->points[p].y);
      bb_z_max = std::min(bb_z_max, fc.cloud->points[p].z);
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
  
  //Check for no self loops
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








float compute_face_similarity(int ai, int bi, FaceGraph const& graph) {
    FaceClusterProps a = graph.get_node_props(ai);
    FaceClusterProps b = graph.get_node_props(bi);

	  float bbox_diff = abs(a.geomdesc.bb_x - b.geomdesc.bb_x) +
		  abs(a.geomdesc.bb_y - b.geomdesc.bb_y) +
		  abs(a.geomdesc.bb_z - b.geomdesc.bb_z);

	  bbox_diff /= min(a.bbox_edge_sum, b.bbox_edge_sum) * 10;
	  //0 = perfect match
	  //1 = difference is one tenth of avg bbox edge sum lengths.

	  float dp = face_geom_desc_[a].o_x * face_geom_desc_[b].o_x +
		  face_geom_desc_[a].o_y * face_geom_desc_[b].o_y +
		  face_geom_desc_[a].o_z * face_geom_desc_[b].o_z;

	  if (dp > 1) dp = 1;

	  float o_diff = acos(dp) / (15.0f / 180.0f * M_PI);
	  //0 = perfect match
	  //1 = difference in orientation is 15 degrees


    float sz_min = min(a.cloud->size(), b.cloud->size());
    float sz_max = max(a.cloud->size(), b.cloud->size());

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
float compute_neighbourhood_similarity(int ai, int bi, FaceGraph const& graph) {
  std::set<int> const& n_min = graph.adjacent(ai).size() <= graph.adjacent(bi).size() : graph.adjacent(ai) : graph.adjacent(bi);
  std::set<int> const& n_max = graph.adjacent(ai).size() > graph.adjacent(bi).size() : graph.adjacent(ai) : graph.adjacent(bi);
  float avgsim = 0;

  if (n_min.empty() || n_max.size() - n_min.size() > 2) {
    return 0;
  }

  std::vector<std::tuple<float, int, int>> corr_sim;
  
  for (auto it1 = n_min.begin(); it1 != n_min.end(); ++it1) {
    for(auto it2 = n_max.begin(); it2 != n_max.end(); ++it2) {
      float s = compute_face_similarity(graph.get_node_props(*it1), graph.get_node_props(*it2));
      corr_sim.push_back(std::make_tuple(s, *it1, *it2));
    }
  }
  
  std::sort(corr_sim.begin(), corr_sim.end(), std::greater<std::tuple<float, int, int>>());
  
  std::vector<bool> seen_face_min(graph.nodes().size());
  std::vector<bool> seen_face_max(graph.nodes().size());
  
  for(int i=0; i<corr_sim.size(); i++) {
    int minface = std::get<1>(corr_sim[i]);
    int maxface = std::get<2>(corr_sim[i]);
    bool rem = false;
    
    if(seen_face_min[minface])
      rem = true;
    else
      seen_face_min[minface] = true;
    
    if(seen_face_max[maxface])
      rem = true;
    else
      seen_face_max[maxface] = true;
    
      if(rem)
        corr_sim[i] = std::make_tuple(0.0f, 0, 0);
  }
  
  std::remove(corr_sim.begin(), corr_sim.end(), std::make_tuple(0.0f, 0, 0));
    
  assert(corr_sim.size() == n_min.size());
    
  float avgsim = 0;
  for(int i=0; i<corr_sim.size(); i++) {
    avgsim += std::get<0>(corr_sim[i]);
  }
  avgsim /= corr_sim.size();
    
  return avgsim;
}




void do_bad_node_replacement( FaceGraph const& graph,
                              float p_gr_nsim,
                              float p_nsim_co,
                              float p_gr_best_face_sim) {
  

  std::vector<std::tuple<int,int>> replacements; //Replace the first cluster with the second.
  
   
  for(auto c = graph.nodes().begin(); c != graph.nodes().end(); ++it) {
    
    std::vector<std::tuple<int, float, float>> sim_face_neighbourhood_list;
    
    for(auto d = graph.nodes().begin(); d != graph.nodes.end(); ++it) {
      if(c == d)
        continue;
      
      float neighbourhood_sim = compute_neighbourhood_similarity(c, d, graph);
      
      if(neighbourhood_sim >= p_gr_nsim)
        sim_face_neighbourhood_list.push_back(std::make_tuple(d, compute_face_similarity(c, d, graph), neighbourhood_sim));
    }
    

    bool sim_neighbourhoods_cores_coherent = true;
    
    for(int i=0; i<sim_face_neighbourhood_list.size(); i++) {
      for(int j=i+1; j<sim_face_neighbourhood_list.size(); j++) {
        if(compute_face_similarity(std::get<0>(sim_face_neighbourhood_list[i]), std::get<0>(sim_face_neighbourhood_list[j]), graph) < p_nsim_co) {
          sim_neighbourhoods_cores_coherent = false;
          break;
        }
      }
    }
    
    if(!sim_neighbourhoods_cores_coherent)
      continue;
    
    float target_best_core_sim = 0;
    
    for(int i=0; i<sim_face_neighbourhood_list.size(); i++) {
      if(std::get<1>(sim_face_neighbourhood_list[i]) > target_best_core_sim) {
        target_best_core_sim = std::get<1>(sim_face_neighbourhood_list[i]);
      }
    }
    
    if(target_best_core_sim >= p_gr_best_face_sim)
      continue;
    
    float best_replacement_sim = 0;
    int best_replacement_idx = -1;
    
    for(int i=0; i<sim_face_neighbourhood_list.size(); i++) {
      if(std::get<2>(sim_face_neighbourhood_list[i]) > best_replacement_sim) {
        best_replacement_sim = std::get<2>(sim_face_neighbourhood_list[i]);
        best_replacement_idx = std::get<0>(sim_face_neighbourhood_list[i]);
      }
    }
    
    replacements.push_back(std::make_tuple(c, best_replacement_idx));
  }
  
  FaceGraph newgraph = graph;
  
  for(int i=0; i<replacements.size(); i++) {
    pcl::PointCloud<PointXYZ>::Ptr pc = pcl::PointCloud<PointXYZ>::Ptr(new pcl::PointCloud<PointXYZ>);
    pcl::PointXYZ src = graph.get_node(std::get<1>(replacements[i])).bbox_centre;
    pcl::PointXYZ dst = graph.get_node(std::get<0>(replacements[i])).bbox_centre;
    
    Eigen::Affine3f t;
    t << dst.x - src.x, dst.y - src.y, dst.z - src.z;
    
    pcl::transformPointCloud(graph.get_node(std::get<1>(replacements[i])).cloud, pc, t);
    newgraph.get_node(std::get<0>(replacements[i])).cloud = pc;
  }
  
  graph = newgraph;
}



//Reconstruct model from graph
void reconstruct_graph( pcl::PointCloud<PointXYZ>::Ptr cloud, 
                        FaceGraph const& graph) {
  
  cloud.clear();
  for(auto it=graph.nodes().begin(); it != graph.nodes().end(); ++it) {
    cloud += graph.get_node(*it).cloud;
  }
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
    ofstream out("adjlist.txt");
    if(!out)
      std::cout << "Could not open adjlist.txt\n";
    else
      out << adjliststr;
  }


  visualize_clusters<pcl::PointXYZ>(cloud, seg.get_clusters(), g_use_coolwarm_vis ? COLOR_MAP_COOLWARM : COLOR_MAP_RAINBOW, 1);

  

  return (0);
}


