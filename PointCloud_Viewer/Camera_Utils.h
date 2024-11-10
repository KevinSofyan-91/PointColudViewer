#pragma once
#include <GL\glew.h>

#include "glm/gtx/transform.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "glm/glm.hpp"
#include "Shader_Utils.h"
#include "lasreader.hpp"


#define VIEWPORT_WIDTH 960
#define VIEWPORT_HEIGHT 540

extern glm::mat4 mvp;
extern glm::mat4 Model;
extern glm::mat4 View;
extern glm::mat4 Projection;

namespace camera_utils
{
	void computeInitialMVP_Matrix(glm::vec3 cameraPosition, glm::vec3 cameraTarget, glm::vec3 cameraUp);
	glm::vec3 getInitalCameraPosition(LASheader &lasreader);
	glm::vec3 getInitalCameraTarget(LASheader &lasreader);
}
