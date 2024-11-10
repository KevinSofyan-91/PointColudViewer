This is a Simple LAS Point Cloud Data Viewer that allows users to visualize point cloud data stored in LAS files. 
It supports reading LAS files and rendering the point cloud data in 3D, using OpenGL for rendering.

Environment
Operating System: Windows
Development Tool: Visual Studio 2022
Development Language: MFC

Features
Loads and visualizes point cloud data from LAS files.
Supports rendering points with RGB color information.
Allows camera movement and interaction for better exploration of point clouds.
Displays points using OpenGL for efficient rendering.
Installation
Prerequisites
Before you begin, ensure that you have the following installed:

Visual Studio 2022: Download here
CMake: Download here
OpenGL: Comes pre-installed with Visual Studio and the graphics drivers.
GLEW: OpenGL Extension Wrangler Library for handling OpenGL extensions.
GLFW: A library for creating windows and handling user input.
LASlib: A C++ library for reading and writing LAS files.
Setting up the Project
Clone the repository to your local machine:

bash
Copy code
git clone https://github.com/KevinSofyan-91/PointCloudViewer.git
cd PointCloudViewer
Open the project in Visual Studio 2022.

Ensure that the following libraries are correctly set up in your project:

GLEW
GLFW
LASlib
Set the correct paths to the libraries in the project settings for linking and include directories.

Building the Project
Open the CMakeLists.txt file in Visual Studio to configure the project.

Build the project by selecting Build > Build Solution from the top menu, or by pressing Ctrl + Shift + B.

Once the build is successful, you can run the application directly from Visual Studio.

Usage
Running the Viewer
After compiling the program, run the executable from Visual Studio or the terminal.

The viewer will prompt you to select a LAS file. You can do this using a file dialog that appears when the program is launched.

After selecting a file, the point cloud will be loaded and rendered in a 3D view.

Use the mouse to interact with the viewer:

Left-click and drag: Rotate the camera.
Scroll wheel: Zoom in and out.
Right-click: Move the camera.
Supported LAS File Format
The viewer supports LAS 1.0 and LAS 1.4 formats.
RGB color information is extracted from the LAS file and visualized in the viewer.
Features and Functionality
Load LAS File: Open a point cloud dataset in LAS format.
3D Point Cloud Visualization: Render points using OpenGL with color information.
Camera Controls: Move the camera position and adjust the viewing angle to explore the point cloud from different perspectives.
Contributing
We welcome contributions to improve the Simple LAS Point Cloud Data Viewer.

If you'd like to contribute, please follow these steps:

Fork the repository.
Create a new branch (git checkout -b feature-xyz).
Commit your changes (git commit -am 'Add feature xyz').
Push to the branch (git push origin feature-xyz).
Create a new pull request.
Please make sure to test your changes thoroughly before submitting a pull request.

License
This project is licensed under the MIT License - see the LICENSE file for details.

Acknowledgements
OpenGL for rendering the point cloud in 3D.
GLEW for managing OpenGL extensions.
GLFW for creating windows and handling user input.
LASlib for reading LAS files and extracting point data.
