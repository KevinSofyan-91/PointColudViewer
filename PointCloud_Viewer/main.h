#pragma once
#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "Dataset_Utils.h"
#include "Camera_Utils.h"
#include "Shader_Utils.h"

#define POINTSIZE	1.5f
#define VSYNC		1

#define WINDOW_NAME					"Point Cloud ViewPort"
#define VERTEX_SHADER_FILEPATH		"C:\\Users\\Administrator\\Point\\vertex.shader"
#define FRAGMENT_SHADER_FILEPATH	"C:\\Users\\Administrator\\Point\\fragment.shader"

unsigned int shader_utils::shader;
unsigned int positionNumberIndices;

glm::mat4 mvp;
glm::mat4 Model;
glm::mat4 View;
glm::mat4 Projection;

// Camera settings
glm::vec3 cameraPosition(0.0f, 0.0f, 0.0f); // Initial camera position
glm::vec3 cameraTarget(0.0f, 0.0f, 0.0f);   // The point the camera looks at
glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);       // Up vector
float radius = 500.0f;

float cameraDistance = 0.0f;

float yaw = -90.0f; // Yaw angle (in degrees)
float pitch = 0.0f; // Pitch angle (in degrees)
float lastX = VIEWPORT_WIDTH / 2.0f;
float lastY = VIEWPORT_HEIGHT / 2.0f;
bool firstMouse = true;
float sensitivity = 2.0f; // Mouse sensitivity

float zoomSpeed = 10.0f;   // Speed of zooming
bool isLeftMouseButtonDown = false; // Track if the left mouse button is pressed

int currentLOD = 1;