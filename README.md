# Face Mesh.Cpp
Face mesh generator using the Face Mesh Mediapipe model with a CPU delegate written in C++ 

- Plain C/C++ implementation with minimal dependencies (Tensorflow Lite + OpenCV)
- Google MediaPipe models without the MediaPipe framework
- Runs on ARM

## API Features
This library offers support for:
- 3D Face Landmarking (468 points)

### 3D Face Landmarking
This is some example code for face landmarking:

```cpp
/* Create instance of FaceMesh detector */
CLFML::FaceMesh::FaceMesh mesh_det;

/* Initialize model interpreter */
mesh_det.load_model(CFML_FACE_MESH_CPU_MODEL_PATH);

/* Load the image into the model and run inference */
mesh_det.load_image(cam_frame);

/* Get the 3D Face landmarks */
std::array<cv::Point3f, CLFML::FaceMesh::NUM_OF_FACE_MESH_POINTS> face_mesh_keypoints = mesh_det.get_face_mesh_points();
```

## Example code
For a full example showcasing both these API functions see the example code in [example/face_mesh_demo/demo.cpp](example/face_mesh_demo/demo.cpp).

## Building with CMake
Before using this library you will need the following packages installed:
- OpenCV
- Working C++ compiler (GCC, Clang, MSVC (2017 or Higher))
- CMake
- Ninja (**Optional**, but preferred)

### Running the examples (CPU)
1. Clone this repo
2. Run:
```bash
cmake . -B build -G Ninja
```
3. Let CMake generate and run:
```bash
cd build && ninja
```
4. After building you can run (linux & mac):
```bash
./face_mesh_demo
```
or (if using windows)
```bat
face_mesh_demo.exe
```

### Using it in your project as library
Add this to your top-level CMakeLists file:
```cmake
include(FetchContent)

FetchContent_Declare(
    face_mesh.cpp
    GIT_REPOSITORY https://github.com/CLFML/Face_Mesh.Cpp
    GIT_TAG main
    # Put the Face_Mesh lib into lib/ folder
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/Face_Mesh.Cpp
)
FetchContent_MakeAvailable(face_mesh.cpp)
...
target_link_libraries(YOUR_EXECUTABLE CLFML::face_mesh)
```
Or manually clone this repo and add the library to your project using:
```cmake
add_subdirectory(Face_Mesh.Cpp)
...
target_link_libraries(YOUR_EXECUTABLE CLFML::face_mesh)
```

## Building a ROS2 package with Colcon
Before using this library you will need the following packages installed:
- OpenCV
- ROS2
- ROS CV bridge
- Working C++ compiler (GCC or Clang)
- CMake

### Running the examples (Ubuntu, CPU)

1. Clone this repo:
```
git clone https://github.com/CLFML/Face_Mesh.Cpp.git
```

2. Source your ROS2 installation:

```bash
source /opt/ros/jazzy/setup.bash
```

3. Install the dependencies:
```bash
rosdep install --from-paths src -y --ignore-src
``` 

4. Build the package:

```bash
colcon build --packages-select face_mesh
```

5. Set up the environment:

```bash
source install/setup.bash
```

6. Run the camera node:

```bash
ros2 run v4l2_camera v4l2_camera_node
```

7. In another terminal, run the nodes

```bash
ros2 launch example/ros2/launch.py
```


## Aditional documentation
See our [wiki](https://clfml.github.io/Face_Mesh.Cpp/)...

## Todo
- Add language bindings for Python, C# and Java
- Add support for MakeFiles and Bazel
- Add Unit-tests 
- Add support for the [Face Mesh V2 model](https://storage.googleapis.com/mediapipe-assets/Model%20Card%20MediaPipe%20Face%20Mesh%20V2.pdf)

## License
This work is licensed under the Apache 2.0 license. 

The [face_mesh model](https://drive.google.com/file/d/1QvwWNfFoweGVjsXF3DXzcrCnz-mx-Lha/preview) is also licensed under the Apache 2.0 license.