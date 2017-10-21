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
  
  for(int i=0; i<adjlist.size(); i++) {
    for(int j=0; j<adjlist[i].size(); j++) {
      graph.add_edge(cluster_graph_ids.at(i), cluster_graph_ids.at(adjlist[i][j]));
    }
  }
  
  return graph;
}




//~ float compute_face_similarity(FaceCluster<pcl::PointXYZ> const& a , FaceCluster<pcl::PointXYZ> const& b) {

	  //~ float bbox_diff = abs(a.geomdesc.bb_x - b.geomdesc.bb_x) +
		  //~ abs(a.geomdesc.bb_y - b.geomdesc.bb_y) +
		  //~ abs(a.geomdesc.bb_z - b.geomdesc.bb_z);

	  //~ bbox_diff /= min(a.bbox_edge_sum, b.bbox_edge_sum) * 10;
	  //~ //0 = perfect match
	  //~ //1 = difference is one tenth of avg bbox edge sum lengths.

	  //~ float dp = face_geom_desc_[a].o_x * face_geom_desc_[b].o_x +
		  //~ face_geom_desc_[a].o_y * face_geom_desc_[b].o_y +
		  //~ face_geom_desc_[a].o_z * face_geom_desc_[b].o_z;

	  //~ if (dp > 1) dp = 1;

	  //~ float o_diff = acos(dp) / (15.0f / 180.0f * M_PI);
	  //~ //0 = perfect match
	  //~ //1 = difference in orientation is 15 degrees


    //~ float sz_min = min(a.cloud->size(), b.cloud->size());
    //~ float sz_max = max(a.cloud->size(), b.cloud->size());

    //~ float sz_diff = (sz_max - sz_min) / sz_min * 10;
    

	  //~ if (bbox_diff > 1) {
		  //~ bbox_diff = 1;
	  //~ }
	  //~ if (o_diff > 1) {
		  //~ o_diff = 1;
	  //~ }
    //~ if(sz_diff > 1) {
      //~ sz_diff = 1;
    //~ }

	  //~ return 1 - (bbox_diff + o_diff + sz_diff) / 3.0f;
	//~ }



//~ //Find most similar pairs of neighbours for clusters a and b and return their average similarity.
//~ //The number of pairs is equal to the size of the smaller neighbourhood.
//~ //Each pair has one neighbour of a and one of b. A cluster may only appear once for each neighbourhood.
//~ float neighbourhood_similarity(int a, int b, std::vector<FaceCluster<pcl::PointXYZ>>& faces, std::vector<std::vector<int>>& adjlist) {
  //~ std::vector<int>& n_min = adjlist[a].size() <= adjlist[b].size() ? adjlist[a] : adjlist[b];
  //~ std::vector<int>& n_max = adjlist[a].size() > adjlist[b].size() ? adjlist[a] : adjlist[a];
  //~ float avgsim = 0;

  //~ if (n_min.empty() || n_max.size() - n_min.size() > 2) {
    //~ return 0;
  //~ }

  //~ std::vector<std::tuple<float, int, int>> corr_sim;
  
  //~ for (int i = 0; i < n_min.size(); i++) {
    //~ for (int j = 0; j < n_max.size(); j++) {
      //~ float s = faces[n_min[i]].compute_similarity(faces[n_max[j]]);
      //~ corr_sim.push_back(std::make_tuple(s, i, j));
    //~ }
  //~ }
  
  //~ std::sort(corr_sim.begin(), corr_sim.end(), std::greater<std::tuple<float, int, int>>());
  
  //~ std::vector<bool> seen_face_min(faces.size());
  //~ std::vector<bool> seen_face_max(faces.size());
  
  //~ for(int i=0; i<corr_sim.size(); i++) {
    //~ int minface = std::get<1>(corr_sim[i]);
    //~ int maxface = std::get<2>(corr_sim[i]);
    //~ bool rem = false;
    
    //~ if(seen_face_min[minface])
      //~ rem = true;
    //~ else
      //~ seen_face_min[minface] = true;
    
    //~ if(seen_face_max[maxface])
      //~ rem = true;
    //~ else
      //~ seen_face_max[maxface] = true;
    
      //~ if(rem)
        //~ corr_sim[i] = std::make_tuple(0.0f, 0, 0);
  //~ }
  
  //~ std::remove(corr_sim.begin(), corr_sim.end(), std::make_tuple(0.0f, 0, 0));
    
  //~ assert(corr_sim.size() == n_min.size());
    
  //~ float avgsim = 0;
  //~ for(int i=0; i<corr_sim.size(); i++) {
    //~ avgsim += std::get<0>(corr_sim[i]);
  //~ }
  //~ avgsim /= corr_sim.size();
    
  //~ return avgsim;
//~ }



//~ //For each cluster, produce a list of different clusters that are have similar neighbourhoods.
//~ void produce_replacement_suggestions( std::vector<FaceCluster<pcl::PointXYZ>>& faces, 
                                      //~ std::vector<std::vector<int>>& adjlist, 
                                      //~ std::vector<std::vector<int>>& replacement_list,
                                      //~ float p_gr_nsimsug_th) {
  
  //~ /*
   //~ * replacement_list.resize(faces.size())
   //~ * 
   //~ * For each cluster c in faces:
   //~ *  for each cluster d in faces s.t. c != d:
   //~ *    if sim(c,d) > some_threshold:
   //~ *      replacement_list[c].push_back(d)
   //~ * 
   //~ */
   
  //~ replace_list.resize(faces.size());
   
  //~ for(int i=0; i<faces.size(); i++) {
    //~ for(int j=0; j<faces.size(); j++) {
      //~ if(i == j)
       //~ continue;
    
    //~ float s = neighbourhood_similarity(i, j, faces, adjlist);
    
    //~ if(s >= p_gr_nsimsug_th)
      //~ replace_list[i].push_back(j);
    //~ }
  //~ }
//~ }


//~ float compute_min_sim_amongst_list(std::vector<int>& list, std::vector<FaceCluster<pcl::PointXYZ>>& faces) {
  //~ float minsim = 1;
  
  //~ for(int i=0; i<list.size(); i++) {
    //~ for(int j=i+1; j<list.size(); j++) {
      //~ float s = compute_face_similarity(faces[list[i]], faces[list[j]]);
      
      //~ if(s < minsim)
        //~ minsim = s;
    //~ }
  //~ }
  
  //~ return minsim;
//~ }


//~ float compute_max_sim_from_list(int t, std::vector<int>& list, std::vector<FaceCluster<pcl::PointXYZ>>& faces) {
  //~ float maxsim = 0;
  
  //~ for(int i=0; i<list.size(); i++) {
    //~ float s = compute_face_similarity(faces[t], faces[list[i]]);
    //~ if(s > maxsim)
      //~ maxsim = s;
  //~ }
  //~ return maxsim;
//~ }


//~ //For each cluster's suggestion list, determine if that cluster should be replaced, and if so, can it be replaced.
//~ //The condition for replacement is that all suggestions are sufficiently similar to each other, and the target node is sufficiently dissimilar to the suggestions.
//~ void check_should_replace(  std::vector<FaceCluster<pcl::PointXYZ>>& faces, 
                            //~ std::vector<std::vector<int>>& replacement_list, 
                            //~ std::vector<int>& clusters_to_replace,
                            //~ float p_gr_suglistsim_min,
                            //~ float p_gr_simtosuglist_max) {
  
  //~ /*
   //~ * For each cluster c in faces:
   //~ *  s = compute minimum similarity between any two clusters in replacement_list[c]
   //~ *  if s is > some_threshold:
   //~ *    t = compute maximum similarity between c and any cluster in replacement_list[c]
   //~ *      if t < some_threshold2:
   //~ *        clusters_to_replace.push_back(c)
   //~ * 
   //~ * 
   //~ * 
   //~ */
   
   //~ clusters_to_replace.clear();
  
  //~ for(int i=0; i<faces.size(); i++) {
    //~ bool should_replace = compute_min_sim_amongst_list(replacement_list[i], faces) > p_gr_suglistsim_min &&
                          //~ compute_max_sim_from_list(i, replacement_list[i], faces) < p_gr_simtosuglist_max;
    //~ if(should_replace)
      //~ clusters_to_replace.push_back(i);
  //~ }
//~ }


//~ int max_neighbourhood_sim_from_list_idx(int t, std::vector<int>& replacement_list, std::vector<FaceCluster<pcl::PointXYZ>>& faces, std::vector<std::vector<int>>& adjlist) {
  //~ float maxsim = 0;
  //~ int maxidx = -1;
  
  //~ for(int i=0; i<replacement_list.size(); i++) {
    //~ float s = neighbourhood_similarity(t, replacement_list[i], faces, adjlist);
    //~ if(s > maxsim) {
      //~ maxsim = s;
      //~ maxidx = i;
    //~ }
  //~ }
  //~ return maxidx;
//~ }


//~ //For each cluster to be replaced, find the most suitable suggestion. Probably just the suggestion with the greatest similarity.
//~ void do_replacements( std::vector<FaceCluster<pcl::PointXYZ>>& faces, 
                      //~ std::vector<std::vector<int>>& adjlist, 
                      //~ std::vector<std::vector<int>>& replacement_list, 
                      //~ std::vector<int>& clusters_to_replace) {
  
  //~ /*
   //~ * For each cluster c in clusters_to_replace:
   //~ *  d = most similar cluster to c in replace_list[c]
   //~ *  faces[c] = faces[d] //faces is a reference to the actual list, this replaces faces[c]
   //~ * 
   //~ */
   
   //~ std::vector<FaceCluster<pcl::PointXYZ>> newfaces(faces);
   
   //~ for(int i=0; i<clusters_to_replace.size(); i++) {
     //~ if(replacement_list[i].size() == 0)
      //~ continue;
     //~ int maxidx = max_neighbourhood_sim_from_list_idx(i, replacement_list[i], faces, adjlist);
     
     //~ newfaces[i] = faces[replacement_list[i][maxidx]];
     //~ newfaces[i].centroid = faces[i].centroid;
   //~ }
   
   //~ faces = newfaces;
//~ }




//~ //Reconstruct model from graph
//~ void reconstruct_graph( pcl::PointCloud<PointXYZ>::Ptr cloud, 
                        //~ std::vector<pcl::PointIndices::Ptr>& cluster_indices,
                        //~ std::vector<FaceCluster<pcl::PointXYZ>>& faces,
                        //~ std::vector<std::vector<int>>& adjlist) {
  //~ /*
   //~ * Edge props has the info needed for correctly positioning the clusters.
   //~ * The position of a cluster refers to the position of its centroid.
   //~ * 
   //~ * Pick some cluster, position it at the origin.
   //~ * cluster_position(some_cluster) = (0,0,0)
   //~ * 
   //~ * until all clusters positioned:
   //~ *  c = some cluster that hasn't been positioned but adjacent to one that has.
   //~ *  d = positioned cluster that is adjacent to d.
   //~ *  cluster_position(c) = cluster_position(d) + edgeprops[(c,d)].centroid_relpos
   //~ * 
   //~ * 
   //~ * pc = new pointcloud
   //~ * 
   //~ * for cluster c in faces:
   //~ *  f = c.points
   //~ *  translate f into position with cluster_position(c). Remember that centroid of f is not at origin, so take that into account.
   //~ *  
   //~ *  pc.add(f)
   //~ * 
   //~ *  return fc
   //~ */
   
   
   
   
  
//~ }


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

  //~ std::vector<FaceCluster<pcl::PointXYZ>> faces;
  //~ std::vector<std::vector<int>> adjlist; 
  //~ std::map<std::tuple<int, int>, FaceEdgeProps> edgeprops;

  //~ build_face_graph(params, cloud, faces, adjlist, edgeprops);

  //~ std::vector<std::vector<int>> node_replacement_suggestions;
  


  //~ for (int i = 0; i < adjlist.size(); i++) {
	  //~ std::cout << "Cluster " << i << ": ";
	  //~ for (int j = 0; j < adjlist[i].size(); j++) {
		  //~ std::cout << adjlist[i][j] << " ";
	  //~ }
	  //~ std::cout << "\n";
  //~ }
  //~ std::cout << "\n";

  //~ // Array for counting votes against each face
  //~ //int *votes = new int(faces.size());
  //~ std::vector<int> voteCount;

  //~ // fill with zeroes
  //~ for (int i = 0; i < faces.size(); i++) {
	  //~ voteCount.push_back(0);
  //~ }

  //~ // tuning factors for selecting node for replacement
  //~ double vote_thresh_nei = 0.85f; // was .8
  //~ double vote_thresh_node = 0.85f; // was 0.7
  //~ double final_vote_thresh = 0.3f; // 0.5 = if 50% of nodes vote against then change the node

  //~ std::vector < std::tuple<float, int, int, int, float>> sim_list;
  //~ std::vector < std::tuple<int, double>> neighbour_sim_vote;

  //~ // fill with zeroes
  //~ for (int i = 0; i < faces.size(); i++) {
	  //~ neighbour_sim_vote.push_back(std::make_tuple(0, 0.0f));
  //~ }

  //~ // label for prints
  //~ std::cout << "Nodes: " << "N'hood " << "N "<< " Node-Sim \n";

  //~ for (int i = 0; i < faces.size(); i++) {
	  
	  //~ for (int j = i+1; j < faces.size(); j++) {	  
		  //~ std::vector<int>& n_min = adjlist[i].size() <= adjlist[j].size() ? adjlist[i] : adjlist[j];
		  //~ std::vector<int>& n_max = adjlist[i].size() > adjlist[j].size() ? adjlist[i] : adjlist[j];
		  //~ float avgsim = 0;

		  //~ if (n_min.empty()) {
			  //~ continue;
		  //~ }

		  //~ for (int k = 0; k < n_min.size(); k++) {
			  //~ float ms = 0;
			  //~ for (int m = 0; m < n_max.size(); m++) {
				  //~ float s = faces[n_min[k]].compute_similarity(faces[n_max[m]]);
				  //~ if (ms < s)
					  //~ ms = s;
			  //~ }
			  //~ avgsim += ms;

		  //~ }
		  //~ avgsim /= n_min.size();

		  //~ float node_sim = faces[i].compute_similarity(faces[j]);
		  //~ std::cout << i << " " << j << ": " << avgsim << " " << n_max.size() - n_min.size() << " " << node_sim << "\n";


		  //~ // Vote if nodes are not similar but the neighbourhood is similar.
		  //~ if (node_sim < vote_thresh_node && avgsim > vote_thresh_nei) {
			  //~ //votes[i]++;
			  //~ //votes[j]++;
			  //~ voteCount.at(i) += 1;
			  //~ voteCount.at(j) += 1;

			  
		  //~ }
		  //~ // neignbourhood vote
		  //~ double oldSim1 = std::get<1>(neighbour_sim_vote[i]);
		  //~ double oldSim2 = std::get<1>(neighbour_sim_vote[j]);
		  //~ if (oldSim1 < avgsim) {
			  //~ std::get<1>(neighbour_sim_vote[i]) = avgsim; // assign new best neighbourhood
			  //~ std::get<0>(neighbour_sim_vote[i]) = j; // assign new best neighbour
		  //~ }
		  //~ if (oldSim2 < avgsim) {
			  //~ std::get<1>(neighbour_sim_vote[j]) = avgsim; // assign new best neighbourhood
			  //~ std::get<0>(neighbour_sim_vote[j]) = i; // assign new best neighbour
		  //~ }

		  //~ sim_list.push_back(std::make_tuple(avgsim, i, j, n_max.size() - n_min.size(), node_sim));
		
	  //~ } 
  //~ }
  
  //~ // Printing the votes per node
 //~ // for (auto i = voteCount.begin(); i != voteCount.end(); ++i){
	//~ //std::cout << *i << " : " << voteCount.at[*i] << "\n";
 //~ // }

  //~ for (int i = 0; i < voteCount.size(); i++) {
	  //~ std::cout << i << " : " << voteCount.at(i) << "\n";
  //~ }

	  
  //~ for (int i = 0; i < voteCount.size(); i++) {
	  //~ if (voteCount.at(i) >= (voteCount.size() * final_vote_thresh)) {
		  //~ std::cout << "Replace: " << i << " with " << std::get<0>(neighbour_sim_vote[i]) << " N'hood Similarity: " << std::get<1>(neighbour_sim_vote[i]) <<"\n";
	  //~ }
	  //~ //std::cout << i << " : " << voteCount.at(i)<< "\n";
  //~ }



	//~ //delete [] votes;
	//~ //votes = nullptr;

  //~ //std::sort(sim_list.begin(), sim_list.end());

  //~ //for (auto it = sim_list.begin(); it != sim_list.end(); ++it) {
	 //~ // std::cout << std::get<1>(*it) << " " << std::get<2>(*it) << " : " << std::get<0>(*it) << " " << std::get<3>(*it) << " " << std::get<4>(*it) << "\n";
  //~ //}
  //~ std::cout << "End of program\n";
  //~ //visualize_clusters<pcl::PointXYZ>(cloud, seg.get_clusters(), g_use_coolwarm_vis ? COLOR_MAP_COOLWARM : COLOR_MAP_RAINBOW, 1);

  

  return (0);
}


