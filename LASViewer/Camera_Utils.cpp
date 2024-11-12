#include "pch.h"
#include "Camera_Utils.h"

/* Compute the model view matrix projection Matrix */
glm::mat4 CameraUtils::computeInitialMVP_Matrix(glm::vec3 cameraPosition, glm::vec3 cameraTarget, glm::vec3 cameraUp, float aspect)
{
	glm::mat4 Projection;
	glm::mat4 Model;
	glm::mat4 View;
	glm::mat4 mvp;

	Projection = glm::perspective(glm::radians(45.f), aspect, 0.1f, 10000.f);

	Model = glm::mat4(1.0f);

	View = glm::lookAt(
		cameraPosition,
		cameraTarget,
		cameraUp
	);

	mvp = Projection * View * Model;
	return mvp;
}


glm::vec3 CameraUtils::getInitalCameraPosition(LASheader& header) {
	float distanceFactor = 2.0f;

	float xCenter = (header.min_x + header.max_x) / 2.0f;
	float yCenter = (header.min_y + header.max_y) / 2.0f;
	float zCenter = (header.min_z + header.max_z) / 2.0f;

	float xSize = header.max_x - header.min_x;
	float ySize = header.max_y - header.min_y;
	float zSize = header.max_z - header.min_z;
	float diagonalLength = sqrt(xSize * xSize + ySize * ySize + zSize * zSize);

	glm::vec3 cameraPosition(xCenter, yCenter, zCenter + distanceFactor * diagonalLength);

	return cameraPosition;
}

glm::vec3 CameraUtils::getInitalCameraTarget(LASheader& header) {
	glm::vec3 target;

	target.x = (header.min_x + header.max_x) / 2.0f;
	target.y = (header.min_y + header.max_y) / 2.0f;
	target.z = (header.min_z + header.max_z) / 2.0f;

	return target;
}