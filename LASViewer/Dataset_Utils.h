#pragma once
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <functional>

#include "lasreader.hpp"

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/features/normal_3d.h>

#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include "vtk-9.3/vtklibproj/src/proj.h"

#define BATCH_SIZE 100000

struct Point_Infos
{
	float x;
	float y;
	float z;
	float intensity;
	float red;
	float green;
	float blue;

	Point_Infos(float x, float y, float z, float intensity, float red, float green, float blue)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->intensity = intensity;
		this->red = red;
		this->green = green;
		this->blue = blue;
	}
};

class DatasetUtils
{
public:
	pcl::PointCloud<pcl::PointXYZRGB> readPointsInfofromLas(LASreader*& lasreader, std::function<int(int)> batchCallback);
	std::vector <float> getCoordinates(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
	std::vector <float> getPointsColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
};