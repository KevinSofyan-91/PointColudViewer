#pragma once
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <functional>

#include "lasreader.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define BATCH_SIZE 1000

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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPointsInfofromLas(LASreader*& lasreader, std::function<int(int)> batchCallback);
	std::vector <float> getCoordinates(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
	std::vector <float> getPointsColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
};

