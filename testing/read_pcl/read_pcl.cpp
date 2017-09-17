#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>



int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  if(argc == 1)
	return 1;

  pcl::PLYReader plyreader;
  
  if (plyreader.read (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s \n", argv[1]);
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << std::endl;



  return (0);
}
