#include "parse.h"
#include <fstream>

#pragma once

int parse_config_file(std::string const& fn, std::map<std::string, std::string>& kv_map) {
	kv_map.clear();

	std::ifstream ifs(fn);
	if (!ifs) {
		std::cout << "Could not open config file " << fn << "\n";
		return -1;
	}

	std::string line;
	int n = 0;

	while (getline(ifs, line)) {
		n++;

		if (line.empty())
			continue;

		size_t p = line.find('=');
		if (p == std::string::npos) {
			std::cout << "Error in config file, line " << n << " : " << line << "\n";
			return -1;
		}

		std::string k = line.substr(0, p);
		std::string v = line.substr(p + 1);
		kv_map[k] = v;
	}

	return 0;
}


void try_parse_param(std::map<std::string, std::string> const& kv_map, std::string const& k, double& v) {
	if (kv_map.count(k) == 0)
		return;
	v = strtod(kv_map.at(k).c_str(), NULL);
}

void try_parse_param(std::map<std::string, std::string> const& kv_map, std::string const& k, float& v) {
	if (kv_map.count(k) == 0)
		return;
	v = (float)strtod(kv_map.at(k).c_str(), NULL);
}

void try_parse_param(std::map<std::string, std::string> const& kv_map, std::string const& k, int& v) {
	if (kv_map.count(k) == 0)
		return;
	v = (int)strtol(kv_map.at(k).c_str(), NULL, 10);
}

void try_parse_param(std::map<std::string, std::string> const& kv_map, std::string const& k, bool& v) {
	if (kv_map.count(k) == 0)
		return;
	v = 0 != strtod(kv_map.at(k).c_str(), NULL);
}

void try_parse_param(std::map<std::string, std::string> const& kv_map, std::string const& k, std::string& v) {
	if (kv_map.count(k) == 0)
		return;
	v = kv_map.at(k);
}
