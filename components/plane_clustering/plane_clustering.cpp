#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include <cassert>
#include <cmath>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include "common.h"
#include "parse.h"
#include "face_graph.hpp"



//Find most similar pairs of neighbours for clusters a and b and return their average similarity.
//The number of pairs is equal to the size of the smaller neighbourhood.
//Each pair has one neighbour of a and one of b. A cluster may only appear once for each neighbourhood.
float neighbourhood_similarity(int a, int b, std::vector<FaceCluster<pcl::PointXYZ>>& faces, std::vector<std::vector<int>>& adjlist) {
  /*std::vector<int>& n_min = adjlist[i].size() <= adjlist[j].size() ? adjlist[i] : adjlist[j];
  std::vector<int>& n_max = adjlist[i].size() > adjlist[j].size() ? adjlist[i] : adjlist[j];
  float avgsim = 0;

  if (n_min.empty() || n_max.size() - n_min.size() > 2) {
    return 0;
  }

  std::vector<std::tuple<float, int, int>> corr_sim;
  
  for (int i = 0; i < n_min.size(); i++) {
    for (int j = 0; j < n_max.size(); j++) {
      float s = faces[n_min[i]].compute_similarity(faces[n_max[k]]);
      corr_sim.push_back(std::make_tuple(s, i, j);
    }
  }
  
  std::sort(corr_sim.begin(), corr_sim.end(), std::greater<std::tuple<float, int, int>>());
  
  std::vector<bool> seen_face_min(faces.size());
  std::vector<bool> seen_face_max(faces.size());
  
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
    
    return avgsim;*/
	return 0;
}




//For each cluster, produce a list of different clusters that are have similar neighbourhoods.
void produce_replacement_suggestions(std::vector<FaceCluster<pcl::PointXYZ>>& faces, std::vector<std::vector<int>>& adjlist, std::vector<std::vector<int>>& replacement_list) {
  
  /*
   * replacement_list.resize(faces.size())
   * 
   * For each cluster c in faces:
   *  for each cluster d in faces s.t. c != d:
   *    if sim(c,d) > some_threshold:
   *      replacement_list[c].push_back(d)
   * 
   */
  
}


//For each cluster's suggestion list, determine if that cluster should be replaced, and if so, can it be replaced.
//The condition for replacement is that all suggestions are sufficiently similar to each other, and the target node is sufficiently dissimilar to the suggestions.
void find_replacements(std::vector<FaceCluster<pcl::PointXYZ>>& faces, std::vector<std::vector<int>>& replacement_list, std::vector<int>& clusters_to_replace) {
  
  /*
   * For each cluster c in faces:
   *  s = compute minimum similarity between any two clusters in replacement_list[c]
   *  if s is > some_threshold:
   *    t = compute maximum similarity between c and any cluster in replacement_list[c]
   *      if t < some_threshold2:
   *        clusters_to_replace.push_back(c)
   * 
   * 
   * 
   */
  
}


//For each cluster to be replaced, find the most suitable suggestion. Probably just the suggestion with the greatest similarity.
void do_replacements(std::vector<FaceCluster<pcl::PointXYZ>>& faces, std::vector<std::vector<int>>& replacement_list, std::vector<int>& clusters_to_replace) {
  
  /*
   * For each cluster c in clusters_to_replace:
   *  d = most similar cluster to c in replace_list[c]
   *  faces[c] = faces[d] //faces is a reference to the actual list, this replaces faces[c]
   * 
   */
}


//Reconstruct model from graph
void reconstruct_graph(...) {
  /*
   * Edge props has the info needed for correctly positioning the clusters.
   * The position of a cluster refers to the position of its centroid.
   * 
   * Pick some cluster, position it at the origin.
   * cluster_position(some_cluster) = (0,0,0)
   * 
   * until all clusters positioned:
   *  c = some cluster that hasn't been positioned but adjacent to one that has.
   *  d = positioned cluster that is adjacent to d.
   *  cluster_position(c) = cluster_position(d) + edgeprops[(c,d)].centroid_relpos
   * 
   * 
   * pc = new pointcloud
   * 
   * for cluster c in faces:
   *  f = c.points
   *  translate f into position with cluster_position(c). Remember that centroid of f is not at origin, so take that into account.
   *  
   *  pc.add(f)
   * 
   *  return fc
   */
  
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

  std::vector<std::vector<int>> node_replacement_suggestions;
  


  for (int i = 0; i < adjlist.size(); i++) {
	  std::cout << "Cluster " << i << ": ";
	  for (int j = 0; j < adjlist[i].size(); j++) {
		  std::cout << adjlist[i][j] << " ";
	  }
	  std::cout << "\n";
  }
  std::cout << "\n";

  // Array for counting votes against each face
  //int *votes = new int(faces.size());
  std::vector<int> voteCount;

  // fill with zeroes
  for (int i = 0; i < faces.size(); i++) {
	  voteCount.push_back(0);
  }

  // tuning factors for selecting node for replacement
  double vote_thresh_nei = 0.85f; // was .8
  double vote_thresh_node = 0.85f; // was 0.7
  double final_vote_thresh = 0.3f; // 0.5 = if 50% of nodes vote against then change the node

  std::vector < std::tuple<float, int, int, int, float>> sim_list;
  std::vector < std::tuple<int, double>> neighbour_sim_vote;

  // fill with zeroes
  for (int i = 0; i < faces.size(); i++) {
	  neighbour_sim_vote.push_back(std::make_tuple(0, 0.0f));
  }

  // label for prints
  std::cout << "Nodes: " << "N'hood " << "N "<< " Node-Sim \n";

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

		  float node_sim = faces[i].compute_similarity(faces[j]);
		  std::cout << i << " " << j << ": " << avgsim << " " << n_max.size() - n_min.size() << " " << node_sim << "\n";


		  // Vote if nodes are not similar but the neighbourhood is similar.
		  if (node_sim < vote_thresh_node && avgsim > vote_thresh_nei) {
			  //votes[i]++;
			  //votes[j]++;
			  voteCount.at(i) += 1;
			  voteCount.at(j) += 1;

			  
		  }
		  // neignbourhood vote
		  double oldSim1 = std::get<1>(neighbour_sim_vote[i]);
		  double oldSim2 = std::get<1>(neighbour_sim_vote[j]);
		  if (oldSim1 < avgsim) {
			  std::get<1>(neighbour_sim_vote[i]) = avgsim; // assign new best neighbourhood
			  std::get<0>(neighbour_sim_vote[i]) = j; // assign new best neighbour
		  }
		  if (oldSim2 < avgsim) {
			  std::get<1>(neighbour_sim_vote[j]) = avgsim; // assign new best neighbourhood
			  std::get<0>(neighbour_sim_vote[j]) = i; // assign new best neighbour
		  }

		  sim_list.push_back(std::make_tuple(avgsim, i, j, n_max.size() - n_min.size(), node_sim));
		
	  } 
  }

  //for (int i = 0; i < voteCount.size(); i++) {
	 // std::cout << i << " : " << voteCount.at(i) << "\n";
  //}

  std::vector<std::tuple<int, int>> replacements;
  for (int i = 0; i < voteCount.size(); i++) {
	  if (voteCount.at(i) >= (voteCount.size() * final_vote_thresh)) {
		  std::cout << "Replace: " << i << " with " << std::get<0>(neighbour_sim_vote[i]) << " N'hood Similarity: " << std::get<1>(neighbour_sim_vote[i]) <<"\n";
		  // do replacement
		  replacements.push_back(std::make_tuple(i, std::get<0>(neighbour_sim_vote[i])));
	  }
	  //std::cout << i << " : " << voteCount.at(i)<< "\n";
  }

  for (int i = 0; i<replacements.size(); i++) {
	  std::cout << "Replaced cluster " << std::get<0>(replacements[i]) << " with " << std::get<1>(replacements[i]) << "\n";

	  pcl::PointCloud<pcl::PointXYZ>::Ptr pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	  int srcNodeIndex = std::get<0>(replacements[i]);
	  int destNodeIndex = std::get<1>(replacements[i]);
	  pcl::PointXYZ src = faces[srcNodeIndex].compute_bbox_centre();
	  pcl::PointXYZ dst = faces[destNodeIndex].compute_bbox_centre();

	  Eigen::Affine3f t = Eigen::Affine3f::Identity();
	  t.translation() << dst.x - src.x, dst.y - src.y, dst.z - src.z;

	  std::cout << t.translation() << "\n";

	  pcl::transformPointCloud<pcl::PointXYZ>(*faces[destNodeIndex].cloud, *pc, t);
	  faces[destNodeIndex].cloud = pc;
  }

  // bad node replacement
  std::vector<pcl::PointIndices::Ptr> cluster_indices;
  cloud->clear();
  cluster_indices.clear();
  for (int i = 0; i < faces.size(); i++) {
	  pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr face = faces[i].cloud;
	  *cloud += *face;
  }


  //for (auto it = graph.nodes().begin(); it != graph.nodes().end(); ++it) {
	 // 

	 // for (int j = cloud->size(); j < cloud->size() + graph.get_node_props(*it).cloud->size(); j++) {
		//  indices->indices.push_back(j);
	 // }

	 // cluster_indices.push_back(indices);
	 // *cloud += *(graph.get_node_props(*it).cloud);

  //}
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = true;

  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped())
  {
  }

	//delete [] votes;
	//votes = nullptr;

  //std::sort(sim_list.begin(), sim_list.end());

  //for (auto it = sim_list.begin(); it != sim_list.end(); ++it) {
	 // std::cout << std::get<1>(*it) << " " << std::get<2>(*it) << " : " << std::get<0>(*it) << " " << std::get<3>(*it) << " " << std::get<4>(*it) << "\n";
  //}
  std::cout << "End of program\n";
  //visualize_clusters<pcl::PointXYZ>(cloud, seg.get_clusters(), g_use_coolwarm_vis ? COLOR_MAP_COOLWARM : COLOR_MAP_RAINBOW, 1);

  

  return (0);
}


