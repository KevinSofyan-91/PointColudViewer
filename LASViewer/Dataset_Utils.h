#pragma once
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <functional>

#include "lasreader.hpp"

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
	std::vector <Point_Infos> readPointsInfofromLas(LASreader*& lasreader, std::function<int(int)> batchCallback);
	std::vector <float> getCoordinates(std::vector <Point_Infos>);
	std::vector <float> getPointsColor(std::vector <Point_Infos>);
};

