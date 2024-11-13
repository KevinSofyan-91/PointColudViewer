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

#define WM_LOADING_COMPLETED (WM_USER + 1) 
#define WM_LOADING_UPDATED (WM_USER + 2) 

class OpenGLRenderer
{
public:
    OpenGLRenderer(HWND hWnd, int width, int height);
    ~OpenGLRenderer();

    void LoadLasPoints(const CString& filePath);
    void SetupRender();
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
    void PostUpdateMessageToUIThread(int progress);


    HWND m_hWnd;
    HDC m_hDC;
    HGLRC m_hRC;

    std::vector<Point_Infos> pointsInfo;

    //Utils Variable
    DatasetUtils dataUtils;
    CameraUtils cameraUtils;
    ShaderUtils shaderUtils;

    //For LOD
    std::vector<GLfloat> lodVertices;
    std::vector<GLfloat> lodColors;
    int currentLOD;
    int prevLOD;

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

    float lastX, lastY;
    bool firstMouse;
    bool isLeftMouseButtonDown;

    int width, height;
    glm::mat4 mvp;
};