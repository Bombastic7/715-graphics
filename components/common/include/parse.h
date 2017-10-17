#include <iostream>
#include <map>
#include <string>

int parse_config_file(std::string const& fn, std::map<std::string, std::string>& kv_map);
void try_parse_param(std::map<std::string, std::string> const& kv_map, std::string const& k, double& v);
void try_parse_param(std::map<std::string, std::string> const& kv_map, std::string const& k, float& v);
void try_parse_param(std::map<std::string, std::string> const& kv_map, std::string const& k, int& v);
void try_parse_param(std::map<std::string, std::string> const& kv_map, std::string const& k, bool& v);
void try_parse_param(std::map<std::string, std::string> const& kv_map, std::string const& k, std::string& v);