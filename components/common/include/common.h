#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

inline bool ends_with(std::string const & value, std::string const & ending)
{
  if (ending.size() > value.size()) return false;
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}



template<typename PointT>
int load_pcd_ply(std::string const& f, typename pcl::PointCloud<PointT>::Ptr cloud) {
  int res;
  if(ends_with(f, ".pcd") && (res = pcl::io::loadPCDFile(f, *cloud)) == 0) {}
  else if(ends_with(f, ".ply") && (res = pcl::io::loadPLYFile(f, *cloud)) == 0) {}
  else {
	  PCL_ERROR("Bad input file. %s\n", f);
      return -1;
  }
  return 0;
}


enum {
  COLOR_MAP_RAINBOW, COLOR_MAP_COOLWARM
};


template<typename T, typename U>
void rainbow_scale_value(T val, U& r, U& g, U& b) {
  if(!isfinite(val)) {
    r = g = b = 255;
    return;
  }
  
  int h = int(val * 256 * 6);
  int x = h % 0x100;

  r = 0, g = 0, b = 0;
  switch (h / 256)
  {
    case 0: r = 255; g = x;       break;
    case 1: g = 255; r = 255 - x; break;
    case 2: g = 255; b = x;       break;
    case 3: b = 255; g = 255 - x; break;
    case 4: b = 255; r = x;       break;
    case 5: r = 255; b = 255 - x; break;
  }
}


//A really quick color map. Simple linear interpolation between (85,72,193), (242,242,242), (177,1,39) for 0, 0.5, 1, respectively.
//If val is not finite, colour is off map.
template<typename T, typename U>
void coolwarm_scale_value(T val, U& r, U& g, U& b) {
  if(!isfinite(val)) {
    r = b = 255;
    g = 0;
    return;
  }
  
  if(val < 0.5) {
    r = 85 + 157 * val;
    g = 72 + 170 * val;
    b = 193 + 49 * val;
  }
  else {
    r = 177 + 65 * (1 - val);
    g = 1 + 241 * (1 - val);
    b = 39 + 203 * (1-val);
  }
}

template<typename U, typename PointT>
void make_rgb_scale(std::vector<U>& vals, typename pcl::PointCloud<PointT>::Ptr cloud_rgb, int map_type) {

  if(vals.size() == 0)
    return;
  
  U val_min, val_max;
  val_min = val_max = vals[0];
  
  for(auto it = vals.begin(); it != vals.end(); ++it) {
    U v = *it;
    if(v < val_min)
      val_min = v;
    if(v > val_max)
      val_max = v;
  }
  
  for(int i=0; i<vals.size(); i++) {
    if(map_type == COLOR_MAP_RAINBOW)
      rainbow_scale_value((vals[i] - val_min) / (val_max - val_min), cloud_rgb->points[i].r, cloud_rgb->points[i].g, cloud_rgb->points[i].b);
    if(map_type == COLOR_MAP_COOLWARM)
      coolwarm_scale_value((vals[i] - val_min) / (val_max - val_min), cloud_rgb->points[i].r, cloud_rgb->points[i].g, cloud_rgb->points[i].b);
  }
}
