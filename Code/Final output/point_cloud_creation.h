#ifndef POINT_CLOUD_CREATION_H
#define POINT_CLOUD_CREATION_H

#include "l_system.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <chrono>
#include <pcl/octree/octree.h>
#include <unordered_map>
#include <string>
#include <cmath>




class PointCloudCreation {
public:
	Grapes grape;
	LSystem lsrule;
	

	std::vector<Node> temp_stem;
	std::vector<Node> temp_berry;
	std::set<int> processed_indices;

	std::chrono::duration<double> Time_KD;
	std::chrono::duration<double> Time_CM;
	std::chrono::duration<double> Time_BS;

	const int num_slices = 100;
	const int image_width = 500;
	const int image_height = 500;
	const float intensity_scale = 10.0f;

	float minX;
	float maxX;
	float minY;
	float maxY;
	float minZ;
	float maxZ;

	float slice_thickness;
	float x_scale;
	float y_scale;
	float z_scale;
	struct Point {
		double x, y, z;
	};

	PointCloudCreation();
	
	void drawCylinder(double r, double h);
	void drawChannel(Node A, Node B, double r);
	void drawBerry(Node A, Node dir, double r);
	void GenerateFullModelPGM(const std::string& filename);
	void drawStem();
	
	void SpaceCalculaterI();
	void GenerateSlicePGMY();
	void GenerateSlicePGMZ();


	void ResultGenerator_KD();

	
};

#endif // POINT_CLOUD_CREATION_H
