#pragma once
#include <vector>
#include <string>
#include <stdexcept>

#include "Shader_Utils.h"
#include "Dataset_Utils.h"
#include "Camera_Utils.h"

#include <GL/glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/features/normal_3d.h>

#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include "vtk-9.3/vtklibproj/src/proj.h"

#define EIGEN_DONT_ALIGN_STATICALLY

#define POINTSIZE	1.5f
#define VSYNC		1

#define VERTEX_SHADER_FILEPATH		".\\shaders\\vertex.shader"
#define FRAGMENT_SHADER_FILEPATH	".\\shaders\\fragment.shader"

#define WM_LOADING_COMPLETED (WM_USER + 1) 
#define WM_LOADING_UPDATED (WM_USER + 2) 
#define WM_LOADING_CANCLED (WM_USER + 3)

#define M_PI 3.1415921587

struct Point {
    float x;
    float y;
    float z;
};

struct LINE {
    float x1, y1, x2, y2;
    std::vector<float> x;
    std::vector<float> y;
    int type;
    bool firstFlag;
    float value;
};

class OpenGLRenderer
{
public:
    OpenGLRenderer(HWND hWnd, int width, int height);
    ~OpenGLRenderer();

    //For Line, Volume Calculation
    std::vector<LINE> lines;

    void OnResize(int width, int height);
    void OnMouseMove(int x, int y);
    void OnLButtonDown(int x, int y);
    void OnLButtonUp();
    void OnMouseWheel(short zDelta);

    void RenderScene();
    void SetupRender();
    void ShowCrossSection(int type);
    void ShowGround();
    void SaveToDFX();
    void SaveToSFC();

    void LoadLasPoints(const CString& filePath);

    void ProcessTriangleMesh();
    Point GetGlobalCoordinate(int mouseX, int mouseY);
    void AddLine(LINE _line);

    void SetPointSize(int size);

private:
    bool ground_flag;

    HWND m_hWnd;
    HDC m_hDC;
    HGLRC m_hRC;

    int pointSize;

    float x_offset, y_offset, z_offset;
    float x_scale, y_scale, z_scale;

    float initialDistance, baseResolution;

    //For LOD
    std::vector<GLfloat> lodVertices;
    std::vector<GLfloat> lodColors;

    GLuint vao, ebo, vboPositions, vboColors, shaderProgram;
    GLuint vboCrossPos, vboCrossColor;
    GLuint vboGroundPos, vboGroundColor;
    GLint MatrixID;

    float minX, minY, minZ, maxX, maxY, maxZ;

    std::vector<float> vertex_position_data;
    std::vector<float> vertex_color_data;

    glm::vec3 cameraPosition;
    glm::vec3 cameraTarget;
    glm::vec3 cameraUp;
    bool cameraLoad;

    float yaw, pitch, roll;
    float sensitivity;
    float radius;

    float lastX, lastY;
    bool firstMouse;
    bool isLeftMouseButtonDown;
    GLdouble globalX, globalY, globalZ;

    int width, height;
    glm::mat4 mvp;

    //Utils Variable
    DatasetUtils dataUtils;
    CameraUtils cameraUtils;
    ShaderUtils shaderUtils;

    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>::Ptr gp3;
    pcl::KdTreeFLANN<pcl::PointXYZRGB, flann::L2_Simple<float>> pointKdTree;
    pcl::PolygonMesh mesh;

    //For point cloud data store
    pcl::PointCloud<pcl::PointXYZRGB> pointsInfo;
    pcl::PointCloud<pcl::PointXYZRGB> crossSection;
    pcl::PointCloud<pcl::PointXYZRGB> groundSection;
    pcl::PointCloud<pcl::PointXYZ> pointsOnlyInfo;

    //For Draw
    std::vector<GLfloat> mesh_vertices; // Store vertex positions
    std::vector<GLfloat> mesh_colors; // Store vertex color
    std::vector<unsigned int> mesh_indices; // Store triangle indices

    std::vector<GLfloat> cross_vertices; // Store vertex positions
    std::vector<GLfloat> cross_color; // Store triangle indices

    std::vector<GLfloat> ground_vertices; // Store vertex positions
    std::vector<GLfloat> ground_color; // Store triangle indices

    //Functions
    void SetupOpenGL();
    void CleanupOpenGL();
    void CleanupBuffers();
    void initalizeValues();
    void LoadShaders();

    float getPointCloudDensity();

    void UpdateCameraDirection();

    void ConvertToJGD2011(float x, float y, float z);
    void FiandAndSelectNearPoints(float x, float y, float z);

    void PostUpdateMessageToUIThread(int progress);
};