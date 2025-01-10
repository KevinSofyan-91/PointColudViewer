
#include "pch.h"
#include "Dataset_Utils.h"
#include "iostream"

using namespace std;

/* Reads the point cloud dataset and saves to a vector struct */
pcl::PointCloud<pcl::PointXYZRGB> DatasetUtils::readPointsInfofromLas(LASreader*& lasreader, std::function<int(int)> batchCallback)
{
	pcl::PointCloud<pcl::PointXYZRGB> pointVector;
	pcl::PointCloud<pcl::PointXYZRGB> tempVector;
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;

	// Set the leaf size (downsampling resolution)
	vg.setLeafSize(1.0f, 1.0f, 1.0f); // Set voxel grid size to 1x1x1

	float x, y, z, red, green, blue;

	size_t totalPoints = lasreader->header.number_of_point_records;
	size_t batch = 0, batchIndex = 0;

	// Read and process the LAS file
	while (lasreader->read_point()) {
		// Access the point data
		LASpoint point = lasreader->point;
		x = point.X * lasreader->header.x_scale_factor + lasreader->header.x_offset;
		y = point.Y * lasreader->header.y_scale_factor + lasreader->header.y_offset;
		z = point.Z * lasreader->header.z_scale_factor + lasreader->header.z_offset;
		red = point.rgb[0];
		green = point.rgb[1];
		blue = point.rgb[2];

		batchIndex++;
		if (batchIndex > BATCH_SIZE) {
			batch++;
			batchIndex = 0;
			batchCallback(static_cast<int>(100.0 / totalPoints * batch * BATCH_SIZE));

			// Apply the voxel grid filter
			vg.setInputCloud(tempVector.makeShared());
			vg.filter(tempVector);

			// Append the filtered points to pointVector
			pointVector.insert(pointVector.end(), tempVector.points.begin(), tempVector.points.end());

			// Clear the tempVector for the next cycle (if needed)
			tempVector.clear();
		}

		// Create a PCL point and set the RGB values
		pcl::PointXYZRGB pcl_point;
		pcl_point.x = x;
		pcl_point.y = y;
		pcl_point.z = z;
		pcl_point.r = static_cast<uint8_t>(red / 65535.0f * 255);
		pcl_point.g = static_cast<uint8_t>(green / 65535.0f * 255);
		pcl_point.b = static_cast<uint8_t>(blue / 65535.0f * 255);

		// Add the point to the cloud
		tempVector.points.push_back(pcl_point);
	}

	return pointVector;
}

/* Extracts the coordinates from each struct in the vector of structs */
std::vector <float> DatasetUtils::getCoordinates(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsInfo)
{
	std::vector <float> coordinates;


	for (auto& pointsInfoStruct : pointsInfo->points)
	{
		coordinates.push_back(pointsInfoStruct.x);
		coordinates.push_back(pointsInfoStruct.y);
		coordinates.push_back(pointsInfoStruct.z);
	}

	return coordinates;
}

/* Extracts the RGB components from each struct in the vector of structs */
std::vector <float> DatasetUtils::getPointsColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsInfo)
{
	std::vector <float> pointsColor;

	for (auto& pointsInfoStruct : pointsInfo->points)
	{
		pointsColor.push_back(pointsInfoStruct.r / 255.0);
		pointsColor.push_back(pointsInfoStruct.g / 255.0);
		pointsColor.push_back(pointsInfoStruct.b / 255.0);
	}

	return pointsColor;
}
