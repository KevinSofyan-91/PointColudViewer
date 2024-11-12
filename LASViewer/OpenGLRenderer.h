#pragma once
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <string>
#include "LASreader.hpp" // Assuming you have LAS reader included
#include "Shader_Utils.h"
#include "Dataset_Utils.h"
#include "Camera_Utils.h"

#define POINTSIZE	1.5f
#define VSYNC		1

#define VERTEX_SHADER_FILEPATH		".\\shaders\\vertex.shader"
#define FRAGMENT_SHADER_FILEPATH	".\\shaders\\fragment.shader"

class OpenGLRenderer
{
public:
    OpenGLRenderer(HWND hWnd, int width, int height);
    ~OpenGLRenderer();

    void LoadLasPoints(const CString& filePath);
    void RenderScene();
    void UpdateCameraDirection();
    void UpdateLod();
    void OnResize(int width, int height);
    void OnMouseMove(int x, int y);
    void OnLButtonDown(int x, int y);
    void OnLButtonUp();
    void OnMouseWheel(short zDelta);

private:
    void SetupOpenGL();
    void CleanupOpenGL();
    void LoadShaders();
    void CleanupBuffers();
    void initalizeValues();

    HWND m_hWnd;
    HDC m_hDC;
    HGLRC m_hRC;

    //Utils Variable
    DatasetUtils dataUtils;
    CameraUtils cameraUtils;
    ShaderUtils shaderUtils;

    GLuint vao, vboPositions, vboColors, shaderProgram;
    GLint MatrixID;

    std::vector<float> vertex_position_data;
    std::vector<float> vertex_color_data;

    glm::vec3 cameraPosition;
    glm::vec3 cameraTarget;
    glm::vec3 cameraUp;

    float yaw, pitch, radius;
    float sensitivity;
    float zoomSpeed;
    float cameraDistance;
    int currentLOD;

    float lastX, lastY;
    bool firstMouse;
    bool isLeftMouseButtonDown;

    int width, height;
    glm::mat4 mvp;
};