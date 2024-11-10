#include "main.h"

//Calculate LOD
void updateLod() {
    cameraDistance = glm::length(cameraPosition);
    if (cameraDistance < 100.0f)
        currentLOD = 1;  // High detail
    else if (cameraDistance < 500.0f)
        currentLOD = 2;  // Medium detail
    else if (cameraDistance < 1000.0f)
        currentLOD = 5;  // Low detail
    else
        currentLOD = 10; // Lowest detail
}

// Update the camera direction based on yaw and pitch
void updateCameraDirection() {
    float yawRad = glm::radians(yaw);
    float pitchRad = glm::radians(pitch);

    glm::vec3 direction;
    cameraPosition.x = cameraTarget.x + radius * cos(yawRad) * cos(pitchRad);
    cameraPosition.y = cameraTarget.y + radius * sin(pitchRad);
    cameraPosition.z = cameraTarget.z + radius * sin(yawRad) * cos(pitchRad);

    updateLod();
    camera_utils::computeInitialMVP_Matrix(cameraPosition, cameraTarget, cameraUp);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    radius -= (float)yoffset * zoomSpeed;

    glm::vec3 direction = glm::normalize(cameraPosition - cameraTarget);  // Camera's direction vector
    cameraPosition = cameraTarget + direction * radius;
    camera_utils::computeInitialMVP_Matrix(cameraPosition, cameraTarget, cameraUp);
}

// Mouse movement callback
void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    if (!isLeftMouseButtonDown) return; // Only process if left mouse button is down

    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xOffset = xpos - lastX;
    float yOffset = lastY - ypos; // Reversed: y-coordinates go from bottom to top
    lastX = xpos;
    lastY = ypos;

    xOffset *= sensitivity;
    yOffset *= sensitivity;

    yaw += xOffset;
    pitch += yOffset;

    // Constrain pitch to prevent the camera from flipping
    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    updateCameraDirection();
}

// Mouse button callback function
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            isLeftMouseButtonDown = true;
            firstMouse = true; // Reset first mouse flag to avoid jump in rotation
        }
        else if (action == GLFW_RELEASE) {
            isLeftMouseButtonDown = false;
        }
    }
}

/* Main function */
int main(int argc, const char* argv[])
{

    std::cout << "Reading Points! Please Wait. " << std::endl;

    // Specify the LAS file path
    LASreadOpener lasOpener;

    // Set the file path for the LAS file
    const char* lasFilePath = "pcl.las";
    lasOpener.set_file_name(lasFilePath);

    LASreader* lasreader = lasOpener.open();

    // Load point cloud data
    const std::vector<data_utils::Point_Infos> pointsInfo = data_utils::readPointsInfofromLas(lasreader);
    const std::vector<float> vertex_position_data = data_utils::getCoordinates(pointsInfo);
    const std::vector<float> vertex_color_data = data_utils::getPointsColor(pointsInfo);

    cameraPosition = camera_utils::getInitalCameraPosition(lasreader->header);
    cameraTarget = camera_utils::getInitalCameraTarget(lasreader->header);

    size_t numPoints = vertex_position_data.size() / 3; // Number of points
    std::cout << "Total Points Loaded: " << numPoints << std::endl;
    std::cout << "Read Completed! Drawing. " << std::endl;

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW!" << std::endl;
        return -1;
    }

    // Create GLFW window
    GLFWwindow* viewPortWindow = glfwCreateWindow(VIEWPORT_WIDTH, VIEWPORT_HEIGHT, WINDOW_NAME, NULL, NULL);
    if (!viewPortWindow) {
        std::cerr << "Failed to create GLFW window!" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(viewPortWindow);

    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW!" << std::endl;
        return -1;
    }
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;

    if (GLEW_ARB_draw_indirect) {
        std::cout << "ARB DSA Supported!" << std::endl;
    }

    // Compute initial MVP matrix    
    camera_utils::computeInitialMVP_Matrix(cameraPosition, cameraTarget, cameraUp);

    // Load shaders
    std::string vertexShaderSource = shader_utils::readShaderFile(VERTEX_SHADER_FILEPATH);
    std::string fragmentShaderSource = shader_utils::readShaderFile(FRAGMENT_SHADER_FILEPATH);
    GLuint shaderProgram = shader_utils::createShaders(vertexShaderSource, fragmentShaderSource);

    // Check if shaders compiled successfully
    GLint success;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER_PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        return -1;
    }

    // Use the shader program
    glUseProgram(shaderProgram);

    // Get the location of the MVP uniform and set it
    GLuint MatrixID = glGetUniformLocation(shaderProgram, "MVP");
    if (MatrixID == -1) {
        std::cerr << "Failed to get uniform location for MVP!" << std::endl;
        return -1;
    }
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);

    // Setup OpenGL buffers
    GLuint vao, vboPositions, vboColors;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vboPositions);
    glGenBuffers(1, &vboColors);

    glBindVertexArray(vao);

    // Position buffer
    glBindBuffer(GL_ARRAY_BUFFER, vboPositions);
    glBufferData(GL_ARRAY_BUFFER, vertex_position_data.size() * sizeof(float), vertex_position_data.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);

    // Color buffer
    glBindBuffer(GL_ARRAY_BUFFER, vboColors);
    glBufferData(GL_ARRAY_BUFFER, vertex_color_data.size() * sizeof(float), vertex_color_data.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(1);

    // Unbind buffers
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Set OpenGL state
    glPointSize(POINTSIZE);
    glEnable(GL_DEPTH_TEST);

    // Main render loop
    glfwSwapInterval(VSYNC);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    
    //set mouse wheel event
    glfwSetScrollCallback(viewPortWindow, scroll_callback);
    glfwSetMouseButtonCallback(viewPortWindow, mouse_button_callback);
    // Set mouse callback
    glfwSetCursorPosCallback(viewPortWindow, mouse_callback);

    glm::vec3 cameraDirection = glm::normalize(cameraPosition - cameraTarget);

    updateCameraDirection();

    while (!glfwWindowShouldClose(viewPortWindow)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Update MVP uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);

        // Draw points
        glBindVertexArray(vao);
        //glDrawArrays(GL_POINTS, 0, numPoints);
        
        // Render points based on currentLOD
        for (size_t i = 0; i < numPoints; i += currentLOD) {
            glDrawArrays(GL_POINTS, static_cast<GLint>(i), 1);
        }
        glBindVertexArray(0);

        glfwSwapBuffers(viewPortWindow);
        glfwPollEvents();
    }


    // Cleanup
    glDeleteBuffers(1, &vboPositions);
    glDeleteBuffers(1, &vboColors);
    glDeleteVertexArrays(1, &vao);
    glDeleteProgram(shaderProgram);

    glfwTerminate();
    return 0;

}

