#include "pch.h"
#include "OpenGLRenderer.h"

#include <GL\glew.h>
#include "glm/gtx/transform.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <glm/gtc/type_ptr.hpp>
#include "glm/glm.hpp"

OpenGLRenderer::OpenGLRenderer(HWND hWnd, int width, int height)
    : m_hWnd(hWnd), width(width), height(height), cameraPosition(0.0f, 0.0f, 0.0f),
    cameraTarget(0.0f, 0.0f, 0.0f), cameraUp(0.0f, 1.0f, 0.0f), yaw(-90.0f), pitch(0.0f),
    radius(500.0f), sensitivity(1.0f), zoomSpeed(10.0f), currentLOD(1), prevLOD(1),
    lastX(static_cast<float>(width) / 2.0f), lastY(static_cast<float>(height) / 2.0f),
    firstMouse(true), isLeftMouseButtonDown(false), x_offset(0.0f), y_offset(0.0f), z_offset(0.0f),
    x_scale(1.0f), y_scale(1.0f), z_scale(1.0f), cameraLoad(false)
{
    // Get device context (DC) for the window to render OpenGL
    m_hDC = ::GetDC(hWnd);

    // Initialize OpenGL
    SetupOpenGL();
}

OpenGLRenderer::~OpenGLRenderer()
{
    CleanupOpenGL();
    CleanupBuffers();
}

//Initialize values
void OpenGLRenderer::initalizeValues() {
    radius = 500.0f;
    yaw = -90.0f;
    pitch = 0.0f;
    sensitivity = 1.0f;
    zoomSpeed = 10.0f;
    currentLOD = 1;
    prevLOD = 1;

    firstMouse = true;
    isLeftMouseButtonDown = false;

    lastX = width / 2.0f;
    lastY = height / 2.0f;
}

void OpenGLRenderer::SetupOpenGL()
{
    PIXELFORMATDESCRIPTOR pfd = {
        sizeof(PIXELFORMATDESCRIPTOR),
        1,
        PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
        PFD_TYPE_RGBA,
        32,
        0, 0, 0, 0, 0, 0,
        0,
        0,
        0,
        0, 0, 0, 0,
        24,
        8,
        0,
        PFD_MAIN_PLANE,
        0,
        0, 0, 0
    };

    int pixelFormat = ChoosePixelFormat(m_hDC, &pfd);
    SetPixelFormat(m_hDC, pixelFormat, &pfd);

    m_hRC = wglCreateContext(m_hDC);
    wglMakeCurrent(m_hDC, m_hRC);

    if (glewInit() != GLEW_OK) {
        MessageBox(NULL, _T("Failed to initialize GLEW"), _T("Error"), MB_OK | MB_ICONERROR);
        return;
    }

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
}

void OpenGLRenderer::CleanupOpenGL()
{
    if (m_hRC) {
        wglMakeCurrent(nullptr, nullptr);
        wglDeleteContext(m_hRC);
        m_hRC = nullptr;
    }

    if (m_hDC) {
        ::ReleaseDC(m_hWnd, m_hDC);
        m_hDC = nullptr;
    }
}

void OpenGLRenderer::LoadShaders()
{
    std::string vertexShaderSource = shaderUtils.readShaderFile(VERTEX_SHADER_FILEPATH);
    std::string fragmentShaderSource = shaderUtils.readShaderFile(FRAGMENT_SHADER_FILEPATH);
    shaderProgram = shaderUtils.createShaders(vertexShaderSource, fragmentShaderSource);

    MatrixID = glGetUniformLocation(shaderProgram, "MVP");
    glUseProgram(shaderProgram);
}


void OpenGLRenderer::LoadLasPoints(const CString& filePath)
{
    LASreadOpener lasOpener;

    int pathLength = WideCharToMultiByte(CP_UTF8, 0, filePath, -1, nullptr, 0, nullptr, nullptr);
    char* path = new char[pathLength];
    WideCharToMultiByte(CP_UTF8, 0, filePath, -1, path, pathLength, nullptr, nullptr);

    lasOpener.set_file_name(path);
    LASreader* lasreader = lasOpener.open();

    if (lasreader == nullptr) {
        std::cout << "LAS File Loading Error. " << std::endl;
        return;
    }

    cameraLoad = false;

    //Get header factor
    x_offset = lasreader->header.x_offset;
    y_offset = lasreader->header.y_offset;
    z_offset = lasreader->header.z_offset;

    x_scale = lasreader->header.x_scale_factor;
    y_scale = lasreader->header.y_scale_factor;
    z_scale = lasreader->header.z_scale_factor;

    // Process the point cloud data...
    pointsInfo = dataUtils.readPointsInfofromLas(lasreader, [this](int progress) {
        PostUpdateMessageToUIThread(progress);
        return 0;  // Make sure to return an int as expected by the callback signature
    });

    pointKdTree.setInputCloud(pointsInfo);

    vertex_position_data = dataUtils.getCoordinates(pointsInfo);
    vertex_color_data = dataUtils.getPointsColor(pointsInfo);

    cameraPosition = cameraUtils.getInitalCameraPosition(lasreader->header);
    cameraTarget = cameraUtils.getInitalCameraTarget(lasreader->header);

    cameraLoad = true;
}

// Function to send points to the main thread via PostMessage
void OpenGLRenderer::PostUpdateMessageToUIThread(int progress) {
    HWND parentHwnd = GetParent(m_hWnd);  // Get the parent HWND

    if (parentHwnd != NULL) {
        // Post a message to the parent window (e.g., WM_USER + 1 as a custom message)
        PostMessage(parentHwnd, WM_LOADING_UPDATED, (WPARAM)progress, (LPARAM)0);
    }
}


void OpenGLRenderer::SetupRender() {
    //Clear all data for new render
    CleanupBuffers();
    initalizeValues();

    // Load shaders
    LoadShaders();

    // Setup OpenGL buffers here...
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vboPositions);
    glGenBuffers(1, &vboColors);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vboPositions);
    glBufferData(GL_ARRAY_BUFFER, vertex_position_data.size() * sizeof(float), vertex_position_data.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, vboColors);
    glBufferData(GL_ARRAY_BUFFER, vertex_color_data.size() * sizeof(float), vertex_color_data.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    UpdateCameraDirection();
}

void OpenGLRenderer::RenderScene()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Update MVP matrix
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);

    glBindVertexArray(vao);

    // Ensure positions and colors are uploaded to the GPU once (only on data changes or first load)
    //if (currentLOD != prevLOD) {
        size_t step = currentLOD;
        lodVertices.clear();
        lodColors.clear();

        for (size_t i = 0; i < vertex_position_data.size(); i += step * 3) {
            lodVertices.push_back(vertex_position_data[i]);
            lodVertices.push_back(vertex_position_data[i + 1]);
            lodVertices.push_back(vertex_position_data[i + 2]);

            // Add color data
            lodColors.push_back(vertex_color_data[i]); // R
            lodColors.push_back(vertex_color_data[i + 1]); // G
            lodColors.push_back(vertex_color_data[i + 2]); // B
        }

        // Set up the position buffer (upload data only when LOD changes)
        glBindBuffer(GL_ARRAY_BUFFER, vboPositions);
        glBufferData(GL_ARRAY_BUFFER, lodVertices.size() * sizeof(GLfloat), lodVertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0); // Position
        glEnableVertexAttribArray(0);

        // Set up the color buffer (upload data only when LOD changes)
        glBindBuffer(GL_ARRAY_BUFFER, vboColors);
        glBufferData(GL_ARRAY_BUFFER, lodColors.size() * sizeof(GLfloat), lodColors.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0); // Color
        glEnableVertexAttribArray(1);

        // Update the previous LOD
        prevLOD = currentLOD;
   // }

    // Draw the current LOD
    glDrawArrays(GL_POINTS, 0, lodVertices.size() / 3);

    // Unbind VAO
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Swap buffers
    SwapBuffers(m_hDC);
}

void OpenGLRenderer::UpdateCameraDirection()
{
    float yawRad = glm::radians(yaw);
    float pitchRad = glm::radians(pitch);

    glm::vec3 direction;
    cameraPosition.x = cameraTarget.x + radius * cos(yawRad) * cos(pitchRad);
    cameraPosition.y = cameraTarget.y + radius * sin(pitchRad);
    cameraPosition.z = cameraTarget.z + radius * sin(yawRad) * cos(pitchRad);
    UpdateLod();
    mvp = cameraUtils.computeInitialMVP_Matrix(cameraPosition, cameraTarget, cameraUp, static_cast<float>(width) / static_cast<float>(height));
    RenderScene();
}

void OpenGLRenderer::UpdateLod()
{
    prevLOD = currentLOD;

    cameraDistance = glm::length(cameraPosition);

    if (cameraDistance < 100.0f)
        currentLOD = 1;
    else if (cameraDistance < 500.0f)
        currentLOD = 2;
    else if (cameraDistance < 1000.0f)
        currentLOD = 5;
    else
        currentLOD = 10;
}

void OpenGLRenderer::OnResize(int newWidth, int newHeight)
{
    width = newWidth;
    height = newHeight;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (GLfloat)width / (GLfloat)height, 0.1f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
}

void OpenGLRenderer::FiandAndSelectNearPoints(float x, float y, float z) {
    pcl::PointXYZRGB searchPoint;
    searchPoint.x = x;  // World coordinates from mouse
    searchPoint.y = y;
    searchPoint.z = z;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius = 5.0f;  // Adjust search radius

    if (pointKdTree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
        // Update colors of searched points
       // Update only the colors of the points found in the search
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
            int idx = pointIdxRadiusSearch[i];
            int colorOffset = idx * 3 * sizeof(GLfloat); // Offset in bytes

            // Set the new color in the vertex_color_data array
            vertex_color_data[idx * 3 + 0] = 1.0f; // R
            vertex_color_data[idx * 3 + 1] = 0.0f; // G
            vertex_color_data[idx * 3 + 2] = 0.0f; // B
        }

        RenderScene();
    }
}

void OpenGLRenderer::GetGlobalCoordinate(int mouseX, int mouseY) {
    if (!cameraLoad) return;

    // Get the viewport
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport); // This gets the window size: [x, y, width, height]

    // Read the depth buffer to get the winZ (depth at the mouse position)
    float winZ;
    glReadPixels(mouseX, viewport[3] - mouseY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

    // Get the modelview, projection matrices from glm
    float aspect = static_cast<float>(width) / static_cast<float>(height);

    glm::mat4 projection = glm::perspective(glm::radians(45.f), aspect, 0.1f, 10000.f);
    glm::mat4 modelview = glm::lookAt(cameraPosition, cameraTarget, cameraUp);

    // Prepare GLdouble arrays to hold the matrix data
    GLdouble modelviewGL[16];
    GLdouble projectionGL[16];

    // Copy the glm matrix data into the GLdouble arrays
    for (int i = 0; i < 16; ++i) {
        modelviewGL[i] = static_cast<GLdouble>(modelview[i / 4][i % 4]);
        projectionGL[i] = static_cast<GLdouble>(projection[i / 4][i % 4]);
    }

    // Use gluUnProject to get the world coordinates from screen-space coordinates (mouse)
    gluUnProject(mouseX, viewport[3] - mouseY, winZ, modelviewGL, projectionGL, viewport, &globalX, &globalY, &globalZ);

   /* globalX = (globalX - x_offset) / x_scale;
    globalY = (globalY - y_offset) / y_scale;
    globalZ = (globalZ - z_offset) / z_scale;*/

    FiandAndSelectNearPoints(globalX, globalY, globalZ);
}

void OpenGLRenderer::OnMouseMove(int x, int y)
{
    GetGlobalCoordinate(x, y);
    // Implement camera movement based on mouse
    if (!isLeftMouseButtonDown) return; // Only process if left mouse button is down

    if (firstMouse) {
        lastX = x;
        lastY = y;
        firstMouse = false;
    }

    float xOffset = x - lastX;
    float yOffset = lastY - y; // Reversed: y-coordinates go from bottom to top
    lastX = x;
    lastY = y;

    xOffset *= sensitivity;
    yOffset *= sensitivity;

    yaw += xOffset;
    pitch += yOffset;

    // Constrain pitch to prevent the camera from flipping
    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    UpdateCameraDirection();
}

void OpenGLRenderer::OnLButtonDown(int x, int y)
{
    isLeftMouseButtonDown = true;
    firstMouse = true; // Reset first mouse flag to avoid jump in rotation
}

void OpenGLRenderer::OnLButtonUp()
{
    isLeftMouseButtonDown = false;
}

void OpenGLRenderer::OnMouseWheel(short zDelta)
{
    // Implement zooming
    // Handle zooming in and out
    if (zDelta > 0) {
        // Zoom in (decrease the radius)
        radius -= zoomSpeed;
    }
    else if (zDelta < 0) {
        // Zoom out (increase the radius)
        radius += zoomSpeed;
    }

    // Ensure the camera doesn't zoom too close or too far
    radius = glm::max(radius, 1.0f);  // Minimum zoom distance (can't go inside target)
    radius = glm::min(radius, 1000.0f);  // Maximum zoom distance (arbitrary max distance)

    // Update camera position based on the new radius
    glm::vec3 direction = glm::normalize(cameraPosition - cameraTarget);  // Camera's direction vector
    cameraPosition = cameraTarget + direction * radius;

    // Update the view matrix (or other calculations depending on your setup)
    UpdateCameraDirection();
}

void OpenGLRenderer::CleanupBuffers()
{
    glDeleteBuffers(1, &vboPositions);
    glDeleteBuffers(1, &vboColors);
    glDeleteVertexArrays(1, &vao);
    glDeleteProgram(shaderProgram);
}