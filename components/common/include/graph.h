#include <algorithm>
#include <map>
#include <stdexcept>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#pragma once


template<typename NodeProps, typename EdgeProps>
class UndirectedMapGraph {
  public:
  
  NodeProps& get_node_props(int i) {
    return nodeprops_.at(i);
  }
  
  EdgeProps& get_edge_props(int a, int b) {
    auto e = std::make_tuple( std::min(a,b), std::max(a,b) );
    return edgeprops_.at(e);
  }
  
  std::set<int> const& nodes() {
    return nodes_;
  }
  
  std::set<std::tuple<int,int>> const& edges() {
    return edges_;
  }
  

  int add_node() {
    nodeprops_[next_node_id_] = NodeProps();
    adjlist_[next_node_id_] = std::set<int>();
    nodes_.insert(next_node_id_);
    
    return next_node_id_++;
  }
  
  void remove_node(int i) {
    if(nodeprops_.count(i) == 0)
       throw std::runtime_error(std::string("Node ") + std::to_string(i) + " does not exist.");
    
    std::vector<std::tuple<int,int>> adjedges;
    
    for(auto it=edgeprops_.begin(); it!=edgeprops_.end(); ++it) {
      if(std::get<0>(*it) == i || std::get<1>(*it) == i)
        adjedges.push_back(*it);
    }
    
    for(auto it=adjedges.begin(); it!=adjedges.end(); ++it)
      remove_edge(std::get<0>(*it), std::get<1>(*it));
    
    nodeprops_.remove(i);
    nodes_.erase(i);
  }
  
  void add_edge(int a, int b, bool ignore_existing = false) {
    auto e = std::make_tuple( std::min(a,b), std::max(a,b) );
    if(edgeprops_.count(e) != 0) {
      if(ignore_existing)
        return;
      throw std::runtime_error(std::string("Edge (") + std::to_string(std::get<0>(e)) + "," + std::to_string(std::get<0>(e)) + ") already exists.");
    }
    
    if(nodeprops_.count(a) == 0)
      throw std::runtime_error(std::string("Node ") + std::to_string(a) + " does not exists.");

    if(nodeprops_.count(b) == 0)
      throw std::runtime_error(std::string("Node ") + std::to_string(b) + " does not exists.");

    edgeprops_[e] = EdgeProps();
    adjlist_.at(std::get<0>(e)).insert(std::get<1>(e));
    adjlist_.at(std::get<1>(e)).insert(std::get<0>(e));
    
    edges_.insert(e);
  }
  
  
  void remove_edge(int a, int b) {
    auto e = std::make_tuple( std::min(a,b), std::max(a,b) );
    if(edgeprops_.count(e) == 0)
      throw std::runtime_error(std::string("Edge (") + std::to_string(std::get<0>(e)) + "," + std::to_string(std::get<0>(e)) + ") does not exist.");
    
    edgeprops_.remove(e);
    adjlist_.at(std::get<0>(e)).erase(std::get<1>(e));
    adjlist_.at(std::get<1>(e)).erase(std::get<0>(e));
    
    edges_.erase(e);
  }
  
  
  std::set<int> const& adjacent(int i) {
    return adjlist_.at(i);
  }
  
  std::string adjlist_str() {
    std::stringstream ss;
    for(auto it1=adjlist_.begin(); it1!=adjlist_.end(); ++it1) {
      ss << it1->first << " ";
      
      for(auto it2=it1->second.begin(); it2!=it1->second.end(); ++it2) {
        ss << *it2 << " ";
      }
      ss << "\n";
    }
    return ss.str();
  }
  
  
  private:
  
  std::set<int> nodes_;
  std::set<std::tuple<int,int>> edges_;
  
  std::map<int, NodeProps> nodeprops_;
  std::map<std::tuple<int,int>, EdgeProps> edgeprops_;
  std::map<int, std::set<int>> adjlist_;
  int next_node_id_;
};
