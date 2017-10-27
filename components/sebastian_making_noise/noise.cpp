//// Artificial noise generator for PLY files with rects only
// written by Sebastian Peters, spet499@aucklanduni.ac.nz
// in Oct 2017 for a COMPSCI 715 project
// Comment: Sorry for bad parameter design
// 
// ToDo: add more possible shapes than rects only
//       better parameter design or INI file
//       circle holes

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply/ply.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/conversions.h>
#include <vector>
#include <fstream> 
#include <pcl/common/random.h>


int main(int argc, char** argv)
{
	std::cout << "Make Noise!" << std::endl << std::endl;

	//////// Loading Data

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
	
	if (argc == 1) {
		std::cout << "USAGE: " 
			<< argv[0] << " input.ply output.ply req_point_distance gauss_sigma gauss_noise_in_normal_direction_only window_creation window_fading" << std::endl
			<< "e.g." << "noise.exe input.ply output.ply 0.01 0.01 yes yes no" << std::endl;
		return 1;
	}
	
	if (pcl::io::loadPolygonFile(argv[1], *mesh) == -1) //* load the file
	{
		PCL_ERROR("couldn't read file %s \n", argv[1]);
		return (-1);
	}

	const float req_point_distance = atof(argv[3]); // point distance (step size) - try 0.01
	const float gauss_sigma = atof(argv[4]); // Gauss sigma - try 0.01
	const float mean = 0; // Gauss mean

	std::cout << "Before: Cloud: w*h= " << cloud->width << " * " << cloud->height << " = " << cloud->width * cloud->height << std::endl;

	//////// Data processing

	pcl::fromPCLPointCloud2(mesh->cloud, *cloud);

	// Normal distributions - one defined from parameters
	pcl::common::NormalGenerator<float> NormalDist(mean, gauss_sigma, 786879);  // mean, sigma, seed

	// Normal distributions - ...and one standard normal dist!
	pcl::common::NormalGenerator<float> StdNormalDist(0, 1, 2766879);  // mean, sigma, seed

	// Normal distributions - ...and one standard normal dist!
	pcl::common::UniformGenerator<float> StdUniformDist(0, 1, 187939);  // min, max, seed

	// Temporary: A virable to count to number of created windows in order to limit them
	int winodw_counter = 0;

	// foreach face
	std::vector<pcl::Vertices, std::allocator<pcl::Vertices>>::iterator face;
	for (face = mesh->polygons.begin(); face != mesh->polygons.end(); ++face)
	{
		++winodw_counter;

		std::cout << "Face-Number: " << *face << std::endl;
		// ToDo - this does not work:
		// skip triangles and deformed flats etc.
		std::cout << "Sizeof of vertices: " << sizeof(face->vertices) << std::endl;
		if(sizeof(face->vertices) != 16) continue;

		// get the 4 points of the face
		pcl::PointXYZ p1 = cloud->points.at(face->vertices[0]);
		pcl::PointXYZ p2 = cloud->points.at(face->vertices[1]);
		pcl::PointXYZ p3 = cloud->points.at(face->vertices[2]);
		pcl::PointXYZ p4 = cloud->points.at(face->vertices[3]);

		// output the point (debugging)
		std::cout << "Point 1: " << p1._PointXYZ::data[0] << " " << p1._PointXYZ::data[1] 
			<< " " << p1._PointXYZ::data[2] << " " << std::endl;
		std::cout << "Point 2: " << p2._PointXYZ::data[0] << " " << p2._PointXYZ::data[1]
			<< " " << p2._PointXYZ::data[2] << " " << std::endl;
		std::cout << "Point 3: " << p3._PointXYZ::data[0] << " " << p3._PointXYZ::data[1]
			<< " " << p3._PointXYZ::data[2] << " " << std::endl;
		std::cout << "Point 4: " << p4._PointXYZ::data[0] << " " << p4._PointXYZ::data[1]
			<< " " << p4._PointXYZ::data[2] << " " << std::endl;

		// add a test point between two others
		// try some fancy functions of Eigen

		// Scalar and cross product:
		//float dot_product = x.dot(y);
		//Eigen::Vector3f z = x.cross(y);
		
		//Eigen::Vector3f e_test;
		//e_test = p1.getVector3fMap() - p2.getVector3fMap();
		//e_test = e_test / 2;

		//Eigen::Vector3f e_new = e_test + p1.getArray3fMap();
		//pcl::PointXYZ p_new;
		//p_new.getArray3fMap() = e_new;

		// next try, this time rather rudimentary and no fancy functions of Eigen

		// Add points in a rectangle face
		
		// vector between p1 and p2 - edge 1
		float xd, yd, zd; 
		xd = p1.x - p2.x;
		yd = p1.y - p2.y;
		zd = p1.z - p2.z;

		// vector between p2 and p3 - edge 2  (if you subtract the other way around it turns the inside out - really funny)
		float xd2, yd2, zd2;
		xd2 = p3.x - p2.x;
		yd2 = p3.y - p2.y;
		zd2 = p3.z - p2.z;

		float distance_length = sqrt(pow(xd, 2) + pow(yd, 2) + pow(zd, 2));
		float distance_length2 = sqrt(pow(xd2, 2) + pow(yd2, 2) + pow(zd2, 2));
		
		std::cout << "Dist lenght 1: " << distance_length << std::endl;
		std::cout << "Dist lenght 2: " << distance_length2 << std::endl;

		int step_count1 = distance_length / req_point_distance; // how many points to add in direction of edge 1
		int step_count2 = distance_length2 / req_point_distance; // how many points to add in direction of edge 2

		//// Normal vector (p_cross) for direction of Gauss Noise 
		// is only calculated if gauss_noise_in_normal argument is = yes otherwise it just stays (1,1,1)
		pcl::PointXYZ p_cross (1,1,1);

		if (std::string(argv[5]) == "yes") {
			std::cout << "Gauss Noise is added in all directions, not only normal direction (gauss_noise_in_normal=yes)" << std::endl;
			Eigen::Vector3f e_cross, e1(xd, yd, zd), e2(xd2, yd2, zd2); // creating vectors in Eigen from the edge-vectors
			e_cross = e1.cross(e2); // calculate the cross product i.e. the normal
			e_cross.normalize(); // normalize the normal
			p_cross.getArray3fMap() = e_cross; // put normal back into a PCL point

			std::cout << "Normal: " << p_cross._PointXYZ::data[0] << " " << p_cross._PointXYZ::data[1]
				<< " " << p_cross._PointXYZ::data[2] << " " << std::endl;
		}


		// Alternative: Manual cross product
		// pcl::PointXYZ p_cross(yd*zd2-zd*yd2, zd*xd2 - xd*zd2, xd*yd2 - yd*xd2); // cross product
				
		// just as a test: Normal distribution output numbers
		//std::cout << "Rnd: " << StdNormalDist.run() << std::endl;
		//std::cout << "Rnd: " << StdNormalDist.run() << std::endl;
		//std::cout << "Rnd: " << StdNormalDist.run() << std::endl;
		//std::cout << "Rnd: " << StdNormalDist.run() << std::endl;
		//std::cout << "Rnd: " << StdNormalDist.run() << std::endl;
		
		// step size for each coordinate in direction 1
		xd /= step_count1;
		yd /= step_count1;
		zd /= step_count1;

		// step size for each coordinate in direction 2
		xd2 /= step_count2;
		yd2 /= step_count2;
		zd2 /= step_count2;

		pcl::PointXYZ p_start = p2; // start at point p2 (one corner of the rect)

		// just as a test: subtract the normal -> everything is moved outside by length of the normal |distance_length1*distance_length2|
		/*p_start.x -= p_cross.x;
		p_start.y -= p_cross.y;
		p_start.z -= p_cross.z;*/


		pcl::PointXYZ p_temp, p_inner_temp;
		for (int j = 1; j < step_count2; ++j) {

			p_temp = p_start; // start at start point - every time

			// go step size in direction 1, add new point, go again, etc.
			for (int i = 1; i < step_count1; ++i) {

				// Move into direction
				p_temp.x += xd;
				p_temp.y += yd;
				p_temp.z += zd;

				//// WINDOW CREATION functionality
				// Do NOT add points if window creation is on (just 'continue' through this loops)
				// the following 'if' MUST be placed AFTER the point movement above! (took 30min to find that out!)
				// WITHOUT fading:
				if ((std::string(argv[6]) == "yes") && (std::string(argv[7]) == "no")
					&& (i < 2.0 / 3.0 * step_count1) 
					&& (i > 1.0 / 3.0 * step_count1)
					&& (j < 2.0 / 3.0 * step_count2)
					&& (j > 1.0 / 3.0 * step_count2)
					&& winodw_counter < 3)
						continue;

				// WITH fading:
				if ((std::string(argv[6]) == "yes") && (std::string(argv[7]) == "yes")
					&& (i < 2.0 / 3.0 * step_count1)
					&& (i > 1.0 / 3.0 * step_count1)
					&& (j < 2.0 / 3.0 * step_count2)
					&& (j > 1.0 / 3.0 * step_count2) 
					&& winodw_counter < 3) {

					// First part is the check whether: 
					// (i minus offset from edge of the hole) > (positive StdUniformDist between 0 and 1)/2.0 * halfway to middle of hole
					// 
					// this is a f***ing formula! 
					if( ( (i - 1.0/3.0*step_count1 >= abs(StdUniformDist.run())/2.0 * 1.0/6.0*step_count1) && (i <= 3.0/6.0*step_count1) )
						|| ( (2.0/3.0*step_count1 - i >= abs(StdUniformDist.run())/2.0 * 1.0/6.0*step_count1) ) && (i >= 3.0/6.0*step_count1))
						
						if (((j - 1.0/3.0*step_count2 >= abs(StdUniformDist.run())/2.0 * 1.0/6.0*step_count2) && (j <= 3.0/6.0*step_count2))
							|| ((2.0/3.0*step_count2 - j >= abs(StdUniformDist.run())/2.0 * 1.0/6.0*step_count2)) && (j >= 3.0/6.0*step_count2))
							
								continue;
				 }


				// Calculate new point
				p_inner_temp.x = p_temp.x + p_cross.x*NormalDist.run();
				p_inner_temp.y = p_temp.y + p_cross.y*NormalDist.run();
				p_inner_temp.z = p_temp.z + p_cross.z*NormalDist.run();

				// add point to cloud
				cloud->push_back(p_inner_temp);
			}

			// move start point by step size in direction 2
			p_start.x += xd2;
			p_start.y += yd2;
			p_start.z += zd2;

		}

		std::cout << "Points per face: step_count1*step_count2= " << step_count1 << " * " << step_count2 
			<< " = " << step_count1 * step_count2 << std::endl;

		std::cout << std::endl;
	}

	std::cout << "After: Cloud: w*h= " << cloud->width << " * " << cloud->height << " = " << cloud->width * cloud->height << std::endl;
	std::cout << "Cloud Points: " << cloud->size() << std::endl;

	//////// Visualization

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	//viewer->addPolygonMesh(*mesh, "meshes", 0);  // this adds the original shape
	viewer->addPointCloud(cloud, "cloud", 0);  // this adds the points

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}

	// Write output file
	pcl::PLYWriter plywriter;

	plywriter.write(argv[2], *cloud);


	return (0);
}


// ---------------------------------------------------
// RUBBISH section


////go to a specific line in a txt file
//std::ifstream& GotoLine(std::ifstream& file, unsigned int num) {
//	file.seekg(std::ios::beg);
//	for (int i = 0; i < num - 1; ++i) {
//		file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
//	}
//	return file;
//}
//
////read a plyfile as a polygonmesh
//void readFacesAndCloudFromPlyFile(std::string filename, pcl::PolygonMesh &faces)
//{
//
//	//PREPARE TO READ FACES
//	faces.polygons.clear();
//	std::cout << "READING FILE " << filename.c_str() << std::endl;
//	ifstream myReadFile;
//	myReadFile.open(filename.c_str(), ios::in);
//	myReadFile.seekg(ios::beg);
//
//	//look for number of vertex
//	int nbVertex = 0, nbFaces = 0, header_end_line = 0;
//	std::string currWord;
//	while (!myReadFile.eof() && nbFaces == 0)
//	{
//		myReadFile >> currWord;
//		if (strcmp("element", currWord.c_str()) == 0)
//		{
//			myReadFile >> currWord;
//			if (strcmp("vertex", currWord.c_str()) == 0)
//				myReadFile >> nbVertex;
//			else if (strcmp("face", currWord.c_str()) == 0)
//				myReadFile >> nbFaces;
//		}
//	}
//	std::cout << "VERTEXES: " << nbVertex << " FACES: " << nbFaces << std::endl;
//
//
//	//check which line does the header end
//	myReadFile.seekg(ios::beg);
//	int cpt_line = 0;
//	char current_line[1024];
//	while (!myReadFile.eof() && header_end_line == 0)
//	{
//		myReadFile.getline(current_line, 1024);
//		cpt_line++;
//		if (strcmp("end_header", current_line) == 0)
//			header_end_line = cpt_line;
//	}
//
//	//go to line Header_limit + nbVertex + 1
//	GotoLine(myReadFile, header_end_line + nbVertex + 1);
//
//	//read nbFaces faces or stop at end of file (that should not happen)
//	cpt_line = 0;
//	int dummy, idx1, idx2, idx3;
//	while (cpt_line < nbFaces && !myReadFile.eof())
//	{
//		myReadFile.getline(current_line, 1024);
//		std::stringstream ss(current_line);
//		pcl::Vertices current_face;
//		ss >> dummy >> idx1 >> idx2 >> idx3;
//		current_face.vertices.push_back(idx1);
//		current_face.vertices.push_back(idx2);
//		current_face.vertices.push_back(idx3);
//		faces.polygons.push_back(current_face);
//		cpt_line++;
//	}
//
//	myReadFile.close();
//
//	//READ CLOUD
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPLYFile(filename.c_str(), *cloud_);
//	pcl::toPCLPointCloud2(*cloud_, faces.cloud);
//	faces.header = faces.cloud.header;
//
//	return;
//}