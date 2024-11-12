#pragma once
#include <GL\glew.h>

#include "glm/gtx/transform.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/glm.hpp"

#include "Shader_Utils.h"

#include "lasreader.hpp"

#define VIEWPORT_WIDTH 500
#define VIEWPORT_HEIGHT 500

class CameraUtils
{
public:
	glm::mat4 computeInitialMVP_Matrix(glm::vec3 cameraPosition, glm::vec3 cameraTarget, glm::vec3 cameraUp, float aspect);
	glm::vec3 getInitalCameraPosition(LASheader& lasreader);
	glm::vec3 getInitalCameraTarget(LASheader& lasreader);
};

