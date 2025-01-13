#include "pch.h"
#include "OpenGLRenderer.h"
#include <fstream>
#include <iostream>


OpenGLRenderer::OpenGLRenderer(HWND hWnd, int width, int height)
    : m_hWnd(hWnd), width(width), height(height), cameraPosition(0.0f, 0.0f, 0.0f),
    cameraTarget(0.0f, 0.0f, 0.0f), cameraUp(0.0f, 1.0f, 0.0f), yaw(-M_PI / 2), pitch(0.0f),
    roll(0.0f), sensitivity(0.1f), radius(10.0f), baseResolution(0.0f), initialDistance(0.0f),
    lastX(static_cast<float>(width) / 2.0f), lastY(static_cast<float>(height) / 2.0f),
    firstMouse(true), isLeftMouseButtonDown(false), x_offset(0.0f), y_offset(0.0f), z_offset(0.0f),
    x_scale(1.0f), y_scale(1.0f), z_scale(1.0f), cameraLoad(false), f_mesh(false), f_cross(false), f_ground(false), 
    f_point(true), cross_type(-1), gp3((new pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal>()))
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
    radius = 10.0f;
    yaw = -M_PI / 2;
    pitch = 0.0f;
    roll = 0.0f;
    sensitivity = 0.1f;

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

    glEnable(GL_POLYGON_SMOOTH);
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

void OpenGLRenderer::CleanupBuffers()
{
    glDeleteBuffers(1, &vboPositions);
    glDeleteBuffers(1, &vboColors);
    glDeleteBuffers(1, &vboGroundPos);
    glDeleteBuffers(1, &vboGroundColor);
    glDeleteBuffers(1, &vboCrossPos);
    glDeleteBuffers(1, &vboCrossColor);
    glDeleteVertexArrays(1, &vao);
    glDeleteProgram(shaderProgram);
}

void OpenGLRenderer::LoadShaders()
{
    std::string vertexShaderSource = shaderUtils.readShaderFile(VERTEX_SHADER_FILEPATH);
    std::string fragmentShaderSource = shaderUtils.readShaderFile(FRAGMENT_SHADER_FILEPATH);
    shaderProgram = shaderUtils.createShaders(vertexShaderSource, fragmentShaderSource);

    MatrixID = glGetUniformLocation(shaderProgram, "MVP");
    glUseProgram(shaderProgram);
}

void OpenGLRenderer::ShowCrossSection(int type) {
    if (cross_type != type) f_cross = true;
    else f_cross = !f_cross;
    if (f_cross) f_point = false;
    else f_point = true;
    f_ground = false;
    f_mesh = false;

    cross_type = type;

    cross_vertices.clear();
    cross_color.clear();

    // Apply a pass-through filter to extract the cross-section
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(pointsInfo.makeShared());
    if (type == 0) {
        pass.setFilterFieldName("y");
    }
    else {
        pass.setFilterFieldName("x");
    }

    pass.setFilterLimits(minZ, maxZ); // Specify the range for the cross-section
    pass.filter(crossSection);

    // Fill the vertices and colors vectors
    for (size_t i = 0; i < crossSection.points.size(); ++i) {
        const pcl::PointXYZRGB& point = crossSection.points[i];

        // Vertex (x, y, z)
        if (type == 0) {
            cross_vertices.push_back(point.x);
            cross_vertices.push_back(0);
            cross_vertices.push_back(point.z);
        }
        else {
            cross_vertices.push_back(0);
            cross_vertices.push_back(point.y);
            cross_vertices.push_back(point.z);
        }

        // Color (r, g, b)
        cross_color.push_back(point.r / 255.0f);  // Normalize to 0-1 range
        cross_color.push_back(point.g / 255.0f);  // Normalize to 0-1 range
        cross_color.push_back(point.b / 255.0f);  // Normalize to 0-1 range
    }
    RenderScene();
}

void OpenGLRenderer::ShowGround() {
    ground_vertices.clear();
    ground_color.clear();

    if (f_ground) f_point = true;
    else f_point = false;
    f_mesh = false;
    f_cross = false;
    f_ground = !f_ground;

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();

    // Semesht segmentation parameters for plane
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(2);  // Threshold for segmenting points into plane
    seg.setInputCloud(pointsInfo.makeShared());
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cerr << "No inliers found. Check segmentation parameters or input cloud." << std::endl;
        return;
    }

    // Extract ground points
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(pointsInfo.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(false); // Extract ground points
    extract.filter(groundSection);

    for (size_t i = 0; i < groundSection.points.size(); ++i) {
        const pcl::PointXYZRGB& point = groundSection.points[i];

        ground_vertices.push_back(point.x);
        ground_vertices.push_back(point.y);
        ground_vertices.push_back(point.z);

        // Color (r, g, b)
        ground_color.push_back(point.r / 255.0f);  // Normalize to 0-1 range
        ground_color.push_back(point.g / 255.0f);  // Normalize to 0-1 range
        ground_color.push_back(point.b / 255.0f);  // Normalize to 0-1 range
    }
    RenderScene();
}

void OpenGLRenderer::SaveToDFX() {
    // Use std::make_shared for consistent memory allocation
    auto projected_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();

    // Set segmentation parameters for plane
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(5);  // Threshold for segmenting points into plane
    seg.setInputCloud(pointsInfo.makeShared());
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cerr << "No inliers found. Check segmentation parameters or input cloud." << std::endl;
        return;
    }

    // Project inliers manually to the plane using the coefficients
    for (const auto& idx : inliers->indices) {
        pcl::PointXYZRGB point = pointsInfo.points[idx];

        // Extract coefficients for plane equation (Ax + By + Cz + D = 0)
        float A = coefficients->values[0];
        float B = coefficients->values[1];
        float C = coefficients->values[2];
        float D = coefficients->values[3];

        // Calculate the distance to the plane
        float distance = (A * point.x + B * point.y + C * point.z + D) /
            std::sqrt(A * A + B * B + C * C);

        // Project the point onto the plane
        pcl::PointXYZRGB projected_point;
        projected_point.x = point.x - distance * A;
        projected_point.y = point.y - distance * B;
        projected_point.z = point.z - distance * C;

        // Add projected point to the new point cloud
        projected_cloud->points.push_back(projected_point);
    }

    if (projected_cloud->empty()) {
        std::cerr << "Projection failed. Ensure the input cloud and coefficients are valid." << std::endl;
        return;
    }

    std::ofstream dxf_file("output.dxf");
    if (!dxf_file.is_open()) {
        std::cerr << "Failed to open output.dxf for writing." << std::endl;
        return;
    }

    dxf_file << "0\nSECTION\n2\nHEADER\n0\nENDSEC\n";
    dxf_file << "0\nSECTION\n2\nTABLES\n0\nENDSEC\n";
    dxf_file << "0\nSECTION\n2\nBLOCKS\n0\nENDSEC\n";
    dxf_file << "0\nSECTION\n2\nENTITIES\n";

    for (const auto& point : projected_cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue; // Skip invalid points
        }
        dxf_file << "0\nPOINT\n8\n0\n10\n" << point.x
            << "\n20\n" << point.y
            << "\n30\n" << point.z << "\n";
    }

    dxf_file << "0\nENDSEC\n0\nEOF\n";
    dxf_file.close();

    std::cout << "DXF file generated successfully." << std::endl;
}

void OpenGLRenderer::SaveToSFC() {
    // Use std::make_shared for consistent memory allocation
    auto projected_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    auto coefficients = std::make_shared<pcl::ModelCoefficients>();
    auto inliers = std::make_shared<pcl::PointIndices>();

    // Set segmentation parameters for plane
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(5);  // Threshold for segmenting points into plane
    seg.setInputCloud(pointsInfo.makeShared());
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cerr << "No inliers found. Check segmentation parameters or input cloud." << std::endl;
        return;
    }

    // Project inliers manually to the plane using the coefficients
    for (const auto& idx : inliers->indices) {
        pcl::PointXYZRGB point = pointsInfo.points[idx];

        // Extract coefficients for plane equation (Ax + By + Cz + D = 0)
        float A = coefficients->values[0];
        float B = coefficients->values[1];
        float C = coefficients->values[2];
        float D = coefficients->values[3];

        // Calculate the distance to the plane
        float distance = (A * point.x + B * point.y + C * point.z + D) /
            std::sqrt(A * A + B * B + C * C);

        // Project the point onto the plane
        pcl::PointXYZRGB projected_point;
        projected_point.x = point.x - distance * A;
        projected_point.y = point.y - distance * B;
        projected_point.z = point.z - distance * C;

        // Add projected point to the new point cloud
        projected_cloud->points.push_back(projected_point);
    }

    if (projected_cloud->empty()) {
        std::cerr << "Projection failed. Ensure the input cloud and coefficients are valid." << std::endl;
        return;
    }

    std::ofstream sfc_file("output.sfc");
    for (const auto& point : projected_cloud->points) {
        sfc_file << point.x << "," << point.y << "," << point.z << "\n";
    }
    sfc_file.close();   

    std::cout << "SFC file generated successfully." << std::endl;
}

float OpenGLRenderer::getPointCloudDensity() {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(pointsOnlyInfo.makeShared());

    float total_distance = 0;
    int num_points = pointsOnlyInfo.size();
    int valid_neighbors = 0;

    for (size_t i = 0; i < num_points; ++i) {
        std::vector<int> neighbors_indices;
        std::vector<float> neighbors_distances;

        // Search for the nearest neighbors
        kdtree.nearestKSearch(pointsOnlyInfo[i], 2, neighbors_indices, neighbors_distances); // 2 for self and the closest neighbor

        if (neighbors_indices.size() > 1) {
            // We add the distance to the closest neighbor (not including the point itself)
            total_distance += neighbors_distances[1];
            valid_neighbors++;
        }
    }

    // Calculate average distance between points
    if (valid_neighbors > 0) {
        return total_distance / valid_neighbors;
    }
    else {
        return 0;
    }
}

void OpenGLRenderer::ProcessTriangleMesh() {
    try
    {
        if (f_mesh) f_point = true;
        else f_point = false;
        f_ground = false;
        f_cross = false;
        f_mesh = !f_mesh;

        pcl::PointCloud<pcl::PointXYZRGB> colored_cloud = pointsInfo;

        mesh.polygons.clear();
        mesh.cloud.data.clear();

        mesh_vertices.clear();
        mesh_colors.clear();
        mesh_indices.clear();

        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(colored_cloud.makeShared());
        pass.setFilterFieldName("z");
        pass.setFilterLimits(minZ, maxZ); // Adjust the range to your needs
        pass.filter(colored_cloud);

        // Step 1: Compute normals
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
        pcl::PointCloud<pcl::Normal> normals;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        tree->setInputCloud(colored_cloud.makeShared());
        n.setInputCloud(colored_cloud.makeShared());
        n.setSearchMethod(tree);
        n.setKSearch(50);  // You may adjust this for performance
        n.compute(normals);

        // Check if normals are computed correctly
        if (normals.points.empty()) {
            std::cerr << "Normal computation failed, normals are empty." << std::endl;
            return;
        }

        // Step 2: Concatenate point cloud and normals
        pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_with_normals;
        pcl::concatenateFields(colored_cloud, normals, cloud_with_normals);

        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
        tree2->setInputCloud(cloud_with_normals.makeShared());

        // Check if the concatenation is successful
        if (cloud_with_normals.points.empty()) {
            std::cerr << "Concatenation failed, cloud_with_normals is empty." << std::endl;
            return;
        }
        // Set the parameters for the triangulation
        gp3->setSearchRadius(50);  // Adjust the search radius depending on your point cloud density
        gp3->setMu(2.5);  // Minimum edge length factor
        gp3->setMaximumNearestNeighbors(100);
        gp3->setMaximumSurfaceAngle(M_PI / 2);  // Maximum surface angle to define the normal
        gp3->setMinimumAngle(M_PI / 18);       // Minimum angle for triangles
        gp3->setMaximumAngle(2 * M_PI / 3);    // Maximum angle for triangles
        gp3->setNormalConsistency(false);       // Whether to enforce consistency on the surface normals
        gp3->setInputCloud(cloud_with_normals.makeShared());
        gp3->setSearchMethod(tree2);
        
        try
        {
            gp3->reconstruct(mesh);

            pcl::PointCloud<pcl::PointXYZRGBNormal> pointCloudNormal;
            pcl::fromPCLPointCloud2(mesh.cloud, pointCloudNormal);

            //Copy point cloud data(vertices)
            for (const auto& point : pointCloudNormal.points) {
                float light_intensity = std::max(0.0f, point.normal_x * 0.577f + point.normal_y * 0.577f + point.normal_z * 0.577f); // Dot product with light direction
                float r = light_intensity * point.r;  // Red component
                float g = light_intensity * point.g;  // Green component (could vary)
                float b = light_intensity * point.b;  // Blue component (could vary)

                mesh_vertices.push_back(point.x);
                mesh_vertices.push_back(point.y);
                mesh_vertices.push_back(point.z);
                mesh_colors.push_back(r / 255.0f); // Red
                mesh_colors.push_back(g / 255.0f); // Green
                mesh_colors.push_back(b / 255.0f); // Blue
                //mesh_colors.push_back(1.0f); // Red
                //mesh_colors.push_back(0.0f); // Green
                //mesh_colors.push_back(0.0f); // Blue
            }

            int cnt = 0;
            // Copy polygon data (indices)
            for (const auto& polygon : mesh.polygons) {
                if (polygon.vertices.size() == 3) { // Ensure the polygon is a triangle
                    mesh_indices.push_back(polygon.vertices[0]);
                    mesh_indices.push_back(polygon.vertices[1]);
                    mesh_indices.push_back(polygon.vertices[2]);
                }
                else {
                    std::cerr << "Non-triangular polygon encountered." << std::endl;
                }
                cnt++;
            }
        }
        catch (const pcl::PCLException& e)
        {
            std::cerr << "PCL Exception: " << e.what() << std::endl;
            return;
        }
        catch (const std::exception& e) {
            std::cerr << "Error during mesh reconstruction: " << e.what() << std::endl;
            return;
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error during mesh reconstruction: " << e.what() << std::endl;
        return;
    }
}

void OpenGLRenderer::LoadLasPoints(const CString& filePath)
{
    try {
        f_ground = false;

        LASreadOpener lasOpener;

        int pathLength = WideCharToMultiByte(CP_UTF8, 0, filePath, -1, nullptr, 0, nullptr, nullptr);
        char* path = new char[pathLength];
        WideCharToMultiByte(CP_UTF8, 0, filePath, -1, path, pathLength, nullptr, nullptr);

        lasOpener.set_file_name(path);
        LASreader* lasreader = lasOpener.open();
        LASheader header = lasreader->header;

        if (lasreader == nullptr) {
            std::cout << "LAS File Loading Error. " << std::endl;
            return;
        }

        cameraLoad = false;

        //Get header factor
        x_offset = header.x_offset;
        y_offset = header.y_offset;
        z_offset = header.z_offset;

        x_scale = header.x_scale_factor;
        y_scale = header.y_scale_factor;
        z_scale = header.z_scale_factor;

        minX = header.min_x;
        minY = header.min_y;
        minZ = header.min_z;

        maxX = header.max_x;
        maxY = header.max_y;
        maxZ = header.max_z;


        double dx = header.max_x - header.min_x;
        double dy = header.max_y - header.min_y;
        double dz = header.max_z - header.min_z;

        initialDistance = std::sqrt(dx * dx + dy * dy + dz * dz);
        baseResolution = std::cbrt(dx * dy * dz / header.number_of_point_records);

        // Process the point cloud data...
        pcl::PointCloud<pcl::PointXYZRGB> readPoints = dataUtils.readPointsInfofromLas(lasreader, [this](int progress) {
            PostUpdateMessageToUIThread(progress);
            return 0;  // Make sure to return an int as expected by the callback signature
        });

        cameraPosition = cameraUtils.getInitalCameraPosition(lasreader->header);
        cameraTarget = cameraUtils.getInitalCameraTarget(lasreader->header);

        cameraLoad = true;

        radius = glm::length(cameraPosition);
        float lodResolution = baseResolution * (initialDistance / radius);

        // Create a PointCloud to hold the downsampled points (centroids)
        pointsInfo.points.clear();
        crossSection.points.clear();
        groundSection.points.clear();

        //pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        //sor.setInputCloud(readPoints.makeShared());
        //sor.setMeanK(50);  // Number of nearest neighbors to use for mean distance estimation
        //sor.setStddevMulThresh(1.0);  // Threshold multiplier for determining outliers
        //sor.filter(pointsInfo);

        pcl::VoxelGrid<pcl::PointXYZRGB> vg;

        // Set the leaf size (downsampling resolution)
        vg.setLeafSize(1.0f, 1.0f, 1.0f); // Set voxel grid size to 1x1x1


        // Apply the voxel grid filter
        vg.setInputCloud(readPoints.makeShared());
        vg.filter(pointsInfo);

        pointKdTree.setInputCloud(pointsInfo.makeShared());


        // Copy only x, y, z fields
        for (size_t i = 0; i < pointsInfo.size(); ++i) {
            pcl::PointXYZ point;
            point.x = pointsInfo.points[i].x;
            point.y = pointsInfo.points[i].y;
            point.z = pointsInfo.points[i].z;
            pointsOnlyInfo.push_back(point);
        }

        lodVertices.clear();
        lodColors.clear();

        for (const auto& point : pointsInfo.points) {
            lodVertices.push_back(point.x);
            lodVertices.push_back(point.y);
            lodVertices.push_back(point.z);

            lodColors.push_back(point.r / 255.0f);
            lodColors.push_back(point.g / 255.0f);
            lodColors.push_back(point.b / 255.0f);
        }
        lasreader->close();
    }
    catch (const std::exception& ex) {

    }
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
    glGenBuffers(1, &ebo);
    glGenBuffers(1, &vboCrossPos);
    glGenBuffers(1, &vboCrossColor);
    glGenBuffers(1, &vboGroundPos);
    glGenBuffers(1, &vboGroundColor);

    UpdateCameraDirection();
}

void OpenGLRenderer::RenderScene()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPointSize(pointSize);

    // Update MVP matrix
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);

    glBindVertexArray(vao);

    if (f_ground) {
        // Set up the position buffer (upload data only when LOD changes)
        glBindBuffer(GL_ARRAY_BUFFER, vboGroundPos);
        glBufferData(GL_ARRAY_BUFFER, ground_vertices.size() * sizeof(GLfloat), ground_vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0); // Position
        glEnableVertexAttribArray(0);

        // Set up the color buffer (upload data only when LOD changes)
        glBindBuffer(GL_ARRAY_BUFFER, vboGroundColor);
        glBufferData(GL_ARRAY_BUFFER, ground_color.size() * sizeof(GLfloat), ground_color.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0); // Color
        glEnableVertexAttribArray(1);

        // Draw the current LOD
        glDrawArrays(GL_POINTS, 0, ground_vertices.size() / 3);
    }
    if(f_point) {
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

        // Draw the current LOD
        glDrawArrays(GL_POINTS, 0, lodVertices.size() / 3);

    }

    if (f_cross) {
        // Set up the position buffer (upload data only when LOD changes)
        glBindBuffer(GL_ARRAY_BUFFER, vboCrossPos);
        glBufferData(GL_ARRAY_BUFFER, cross_vertices.size() * sizeof(GLfloat), cross_vertices.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0); // Position
        glEnableVertexAttribArray(0);

        // Set up the color buffer (upload data only when LOD changes)
        glBindBuffer(GL_ARRAY_BUFFER, vboCrossColor);
        glBufferData(GL_ARRAY_BUFFER, cross_color.size() * sizeof(GLfloat), cross_color.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0); // Color
        glEnableVertexAttribArray(1);

        glDrawArrays(GL_POINTS, 0, cross_vertices.size() / 3);
    }

    if (f_mesh) {
        // Upload vertex data
        glBindBuffer(GL_ARRAY_BUFFER, vboPositions);
        glBufferData(GL_ARRAY_BUFFER, mesh_vertices.size() * sizeof(GLfloat), mesh_vertices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0); // Position
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, vboColors);
        glBufferData(GL_ARRAY_BUFFER, mesh_colors.size() * sizeof(GLfloat), mesh_colors.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0); // Color
        glEnableVertexAttribArray(1);

        glGenBuffers(1, &ebo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_indices.size() * sizeof(GLuint), mesh_indices.data(), GL_STATIC_DRAW);

        //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, mesh_indices.size(), GL_UNSIGNED_INT, 0);
    }

    //glColor3f(1.0f, 0.0f, 0.0f);
    //for (int i = 0; i < lines.size(); i++){
    //    LINE line = lines[i];
    //    if (line.type == 1) {
    //        Point first = GetGlobalCoordinate(line.x1, line.y1);
    //        Point second = GetGlobalCoordinate(line.x2, line.y2);
    //        glBegin(GL_LINES);

    //        glVertex2f(first.x, first.y);  // Start point of the line
    //        glVertex2f(second.x, second.y);  // End point of the line
    //        glEnd();
    //    }
    //}

    // Unbind VAO
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Swap buffers
    SwapBuffers(m_hDC);
}

void OpenGLRenderer::SetPointSize(int size) {
    pointSize = size;
    RenderScene();
}

void OpenGLRenderer::UpdateCameraDirection()
{
    // Calculate the direction vector (view vector)
    glm::vec3 front;
    front.x = cos(yaw) * cos(pitch);
    front.y = sin(pitch);
    front.z = sin(yaw) * cos(pitch);

    // Normalize the front vector
    front = glm::normalize(front);

    // Calculate camera position based on the direction and radius (distance)
    cameraPosition = cameraTarget - front * radius;

    // Compute the right and up vectors based on the front vector and roll
    glm::vec3 right = glm::normalize(glm::cross(front, cameraUp));
    glm::vec3 up = glm::normalize(glm::cross(right, front));

    // Apply roll if needed
    glm::mat4 rollMatrix = glm::rotate(glm::mat4(1.0f), roll, front);
    up = glm::vec3(rollMatrix * glm::vec4(up, 0.0f));

    mvp = cameraUtils.computeInitialMVP_Matrix(cameraPosition, cameraTarget, up, static_cast<float>(width) / static_cast<float>(height));
    RenderScene();
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

            lodColors[idx * 3 + 0] = 1.0f; // R
            lodColors[idx * 3 + 1] = 0.0f; // G
            lodColors[idx * 3 + 2] = 0.0f; // B

            // Set the new color in the vertex_color_data array
           /* pointsInfo.points[idx].r = 1.0f;
            pointsInfo.points[idx].g = 0.0f;
            pointsInfo.points[idx].b = 0.0f;*/
        }

        RenderScene();
    }
}

Point OpenGLRenderer::GetGlobalCoordinate(int mouseX, int mouseY) {

    Point pt;
    pt.x = pt.y = pt.z = 0;
    if (!cameraLoad) return pt;

    // Get the viewport
    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport); // This gets the window size: [x, y, width, height]

    // Read the depth buffer to get the winZ (depth at the mouse position)
    float winZ;
    glReadPixels(mouseX, viewport[3] - mouseY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

    if (winZ < 0.0f) winZ = 0.0f;
    if (winZ >= 1.0f) winZ = 0.9999f;

    if (winZ < 0.0f || winZ > 1.0f) {
        return pt;
    }

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
    //FiandAndSelectNearPoints(globalX, globalY, globalZ);

    std::ofstream file("depth_values.txt", std::ios::app);

    if (file.is_open()) {
        // Write depth value to file
        file << "Depth: " << winZ << " " << globalX << " " << globalY << " " << globalZ << std::endl;
        file.close(); // Close the file
    }
    else {
        std::cerr << "Failed to open file for writing." << std::endl;
    }

    pt.x = globalX;
    pt.y = globalY;
    pt.z = globalZ;

    return pt;
}

void OpenGLRenderer::ConvertToJGD2011(float x, float y, float z) {
    PJ* P_in;
    P_in = proj_create_crs_to_crs(NULL, "EPSG:4326", "EPSG:6668", NULL);


    if (P_in == NULL) {
        std::cerr << "Error creating CRS transformation." << std::endl;
        return;
    }

    CString str1;
    str1.Format(_T("%f %f %f"), x, y, z);
    AfxMessageBox(str1);

    // Transform coordinates from EPSG:4326 to JGD2011 (EPSG:6668)
    PJ_COORD coord_in = proj_coord(x, y, z, 0);
    PJ_COORD coord_out = proj_trans(P_in, PJ_FWD, coord_in);

    float jgd_x = coord_out.xyz.x;
    float jgd_y = coord_out.xyz.y;
    float jgd_z = coord_out.xyz.z;

    CString str;
    str.Format(_T("%f %f %f"), jgd_x, jgd_y, jgd_z);
    AfxMessageBox(str);

    // Clean up
    proj_destroy(P_in);
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

    float xoffset = x - lastX;
    float yoffset = lastY - y; // Reversed: y-coordinates go from bottom to top
    lastX = x;
    lastY = y;

    xoffset *= sensitivity;
    yoffset *= sensitivity;

    float cr = cos(roll), sr = sin(roll);
    yaw += xoffset * cr + yoffset * sr;
    pitch -= yoffset * cr + xoffset * sr;

    // Clamp pitch
    static const float PITCH_CLAMP = M_PI * 0.49999f;
    if (std::fabs(pitch) > PITCH_CLAMP) {
        pitch = PITCH_CLAMP * (pitch > 0.f ? 1.f : -1.f);
        yaw = yaw += M_PI;
        roll = roll += M_PI;
    }

    UpdateCameraDirection();
}

void OpenGLRenderer::AddLine(LINE _line) {
    lines.push_back(_line);
    RenderScene();
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
        radius -= 10.0f;
    }
    else if (zDelta < 0) {
        // Zoom out (increase the radius)
        radius += 10.0f;
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