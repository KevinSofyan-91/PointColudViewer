#include "Dataset_Utils.h"
#include "iostream"

using namespace std;

/* Reads the point cloud dataset and saves to a vector struct */
std::vector <data_utils::Point_Infos> data_utils::readPointsInfofromData()
{
	std::vector <data_utils::Point_Infos> pointsVector;


	float x, y, z, intensity, red, green, blue;

	static std::ifstream fileStream("C:\\Users\\Administrator\\Point\\3.txt");
	
	//auto s = inputFile.peek() == std::ifstream::traits_type::eof();

	if (fileStream.is_open())
	{
		for (std::string line; std::getline(fileStream, line); )
		{
			fileStream >> x >> y >> z >> intensity >> red >> green >> blue;
			pointsVector.push_back(data_utils::Point_Infos{ x, y, z, intensity, red, green, blue });
		}
	}

	fileStream.close();

	return pointsVector;
}

std::vector <data_utils::Point_Infos> data_utils::readPointsInfofromLas(LASreader* & lasreader)
{
	std::vector <data_utils::Point_Infos> pointsVector;
	float x, y, z, intensity, red, green, blue;

    LASheader header = lasreader->header;

    // Read and process the LAS file
    while (lasreader->read_point()) {
        // Access the point data
        LASpoint point = lasreader->point;
		x = point.X * header.x_scale_factor + header.x_offset;
		y = point.Y * header.y_scale_factor + header.y_offset;
		z = point.Z * header.z_scale_factor + header.z_offset;
		intensity = 100;
		red = point.rgb[0];
		green = point.rgb[1];
		blue = point.rgb[2];

		pointsVector.push_back(data_utils::Point_Infos{ x, y, z, intensity, red, green, blue });
    }

    // Close the LAS file
    lasreader->close();

	return pointsVector;
}

/* Extracts the coordinates from each struct in the vector of structs */
std::vector <float> data_utils::getCoordinates(std::vector <data_utils::Point_Infos> pointsInfo)
{
	std::vector <float> coordinates;


	for (auto & pointsInfoStruct : pointsInfo)
	{
		coordinates.push_back(pointsInfoStruct.x);
		coordinates.push_back(pointsInfoStruct.y);
		coordinates.push_back(pointsInfoStruct.z);
	}

	return coordinates;
}

/* Extracts the RGB components from each struct in the vector of structs */
std::vector <float> data_utils::getPointsColor(std::vector <data_utils::Point_Infos> pointsInfo)
{
	std::vector <float> pointsColor;

	for (auto & pointsInfoStruct : pointsInfo)
	{
		pointsColor.push_back(pointsInfoStruct.red / 65535.0f);
		pointsColor.push_back(pointsInfoStruct.green / 65535.0f);
		pointsColor.push_back(pointsInfoStruct.blue / 65535.0f);
	}

	return pointsColor;
}