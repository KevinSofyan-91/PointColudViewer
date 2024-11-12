#include "pch.h"
#include "OpenGLRenderer.h"

OpenGLRenderer::OpenGLRenderer(HWND hWnd, int width, int height)
    : m_hWnd(hWnd), width(width), height(height), cameraPosition(0.0f, 0.0f, 0.0f),
    cameraTarget(0.0f, 0.0f, 0.0f), cameraUp(0.0f, 1.0f, 0.0f), yaw(-90.0f), pitch(0.0f),
    radius(500.0f), sensitivity(1.0f), zoomSpeed(10.0f), currentLOD(1),
    lastX(static_cast<float>(width) / 2.0f), lastY(static_cast<float>(height) / 2.0f),
    firstMouse(true), isLeftMouseButtonDown(false)
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
    //Clear all data for new render
    CleanupBuffers();
    initalizeValues();

    LASreadOpener lasOpener;

    CStringA pathStr(filePath);
    char* path = pathStr.GetBuffer();

    lasOpener.set_file_name(path);
    LASreader* lasreader = lasOpener.open();

    if (lasreader == nullptr) {
        std::cout << "LAS File Loading Error. " << std::endl;
        return;
    }

    // Process the point cloud data...
    // Replace with actual LAS data extraction and conversion logic
    std::vector<Point_Infos> pointsInfo = dataUtils.readPointsInfofromLas(lasreader);
    vertex_position_data = dataUtils.getCoordinates(pointsInfo);
    vertex_color_data = dataUtils.getPointsColor(pointsInfo);

    cameraPosition = cameraUtils.getInitalCameraPosition(lasreader->header);
    cameraTarget = cameraUtils.getInitalCameraTarget(lasreader->header);

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

    glEnable(GL_DEPTH_TEST);

    UpdateCameraDirection();
}

void OpenGLRenderer::RenderScene()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Update MVP matrix
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);

    glBindVertexArray(vao);
    for (size_t i = 0; i < vertex_position_data.size() / 3; i += currentLOD) {
        glDrawArrays(GL_POINTS, static_cast<GLint>(i), currentLOD);
    }
    glBindVertexArray(0);

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
    mvp = cameraUtils.computeInitialMVP_Matrix(cameraPosition, cameraTarget, cameraUp, width / height);
    RenderScene();
}

void OpenGLRenderer::UpdateLod()
{
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

void OpenGLRenderer::OnMouseMove(int x, int y)
{

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