cmake_minimum_required(VERSION 3.1...3.14)

project(face_mesh VERSION 3.11.5 LANGUAGES CXX)

set(CLFML_FACE_MESH_MAIN_PROJECT OFF)

# Optionally build the examples
option(CLFML_FACE_MESH_BUILD_EXAMPLE_PROJECTS "Build example projects" ON)

# Optionally enable ROS2 package build (Requires ROS2 jazzy to be installed!)
option(CLFML_ROS2_PACKAGE_BUILD "Build a ROS2 package" OFF)

if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(CMAKE_CXX_STANDARD 17)
  set(CLFML_FACE_MESH_MAIN_PROJECT ON)
else()
  set(CLFML_FACE_MESH_BUILD_EXAMPLE_PROJECTS OFF)
endif()

if (POLICY CMP0077)
  # Allow CMake 3.13+ to override options when using FetchContent / add_subdirectory.
  cmake_policy(SET CMP0077 NEW)
endif()

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  cmake_policy(SET CMP0135 NEW)
endif()

find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/face_mesh.cpp)
add_library(CLFML::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/src)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if (TARGET tensorflow-lite)
  target_link_libraries(${PROJECT_NAME} tensorflow-lite)
else()

  if(${CMAKE_SYSTEM_NAME} STREQUAL "Windows" OR CLFML_FACE_MESH_COMPILE_TENSORFLOW_FROM_SOURCE)
    include(FetchContent)
    FetchContent_Declare(
        tensorflow_rel_package
        URL https://github.com/tensorflow/tensorflow/archive/refs/tags/v2.16.1.zip
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/tensorflow
        SOURCE_SUBDIR tensorflow/lite
    )
    FetchContent_MakeAvailable(tensorflow_rel_package)
    else()
    # Check if we can use any of the precompiled tensorflow lite packages
    # https://github.com/CLFML/TensorFlow_Lite_Compiled
    include(FetchContent)
    if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "arm64")
      FetchContent_Declare(
        tensorflow_compiled_rel_package
        URL https://github.com/CLFML/TensorFlow_Lite_Compiled/releases/download/v2.16.1/tensorflow_linux_generic_aarch64.zip
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/tensorflow
      )
    else()
      # Get the CPU compatible instructionset(s)
      execute_process(
        COMMAND lscpu
        OUTPUT_VARIABLE CLFML_FACE_MESH_CPU_INFO
        OUTPUT_STRIP_TRAILING_WHITESPACE
      )
      if(CLFML_FACE_MESH_CPU_INFO MATCHES "avx2")
        message(STATUS "CLFML Face_Mesh.Cpp: CPU supports AVX2")
        # Download the AVX2 compatible precompiled tensorflow lite package
        FetchContent_Declare(
          tensorflow_compiled_rel_package
          URL https://github.com/CLFML/TensorFlow_Lite_Compiled/releases/download/v2.16.1/tensorflow_linux_avx2_x86_64.zip
          SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/tensorflow
        )
        FetchContent_MakeAvailable(tensorflow_compiled_rel_package)
        if(CLFML_ROS2_PACKAGE_BUILD)
          install(FILES external/tensorflow/lib/libtensorflowlite.so DESTINATION lib/${PROJECT_NAME})
        endif()
 
      elseif(CLFML_FACE_MESH_CPU_INFO MATCHES "avx")
        message(STATUS "CLFML Face_Mesh.Cpp: CPU supports AVX")
        # Download the AVX compatible precompiled tensorflow lite package
        FetchContent_Declare(
          tensorflow_compiled_rel_package
          URL https://github.com/CLFML/TensorFlow_Lite_Compiled/releases/download/v2.16.1/tensorflow_linux_generic_x86_64.zip
          SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/tensorflow
        )
        FetchContent_MakeAvailable(tensorflow_compiled_rel_package)
        if(CLFML_ROS2_PACKAGE_BUILD)
          install(FILES external/tensorflow/lib/libtensorflowlite.so DESTINATION lib/${PROJECT_NAME})
        endif()
      else()
        message(STATUS "CLFML Face_Mesh.Cpp: CPU doesn't support AVX or AVX2; Compiling from source...")
        # Download and build from source
        FetchContent_Declare(
        tensorflow_rel_package
        URL https://github.com/tensorflow/tensorflow/archive/refs/tags/v2.16.1.zip
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/tensorflow
        SOURCE_SUBDIR tensorflow/lite
       )
        FetchContent_MakeAvailable(tensorflow_rel_package)
      endif()
    endif()
  endif()
  target_link_libraries(${PROJECT_NAME} tensorflow-lite)
endif()

set(FACE_MESH_CPU_MODEL_PATH ${CMAKE_CURRENT_LIST_DIR}/models/CPU/face_mesh.tflite)
target_compile_definitions(${PROJECT_NAME} PUBLIC -DCLFML_FACE_MESH_CPU_MODEL_PATH="${FACE_MESH_CPU_MODEL_PATH}")

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

if (CLFML_ROS2_PACKAGE_BUILD)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(cv_bridge REQUIRED)
  target_link_libraries(${PROJECT_NAME} ${cv_bridge_LIBRARIES})
  add_executable(face_mesh_node ${CMAKE_CURRENT_LIST_DIR}/bindings/ros2/face_mesh_node.cpp)
  ament_target_dependencies(face_mesh_node
    rclcpp
    sensor_msgs
    cv_bridge
    std_msgs
    geometry_msgs
    std_srvs
    OpenCV
  )
  target_link_libraries(face_mesh_node face_mesh)
  install(TARGETS
    face_mesh_node
    DESTINATION lib/${PROJECT_NAME})
endif()


if(CLFML_FACE_MESH_BUILD_EXAMPLE_PROJECTS)
FetchContent_Declare(
  face_detector.cpp
  GIT_REPOSITORY https://github.com/CLFML/Face_Detector.Cpp.git
  GIT_TAG main
)
FetchContent_MakeAvailable(face_detector.cpp)
if (CLFML_ROS2_PACKAGE_BUILD)
    add_executable(face_mesh_viewer ${CMAKE_CURRENT_LIST_DIR}/example/ros2/face_mesh_viewer.cpp)
    ament_target_dependencies(face_mesh_viewer
        rclcpp
        sensor_msgs
        cv_bridge
        std_msgs
        geometry_msgs
        std_srvs
        OpenCV
      )
    install(TARGETS
      face_mesh_viewer
      DESTINATION lib/${PROJECT_NAME})
  else ()
  add_executable(face_mesh_demo ${CMAKE_CURRENT_LIST_DIR}/example/face_mesh_demo/demo.cpp)
  target_link_libraries(face_mesh_demo PUBLIC CLFML::${PROJECT_NAME} CLFML::face_detector)
  endif()
endif()


if (CLFML_ROS2_PACKAGE_BUILD)
  install(FILES bindings/ros2/set_ld_path.sh DESTINATION lib/${PROJECT_NAME})
  ament_package()
endif()