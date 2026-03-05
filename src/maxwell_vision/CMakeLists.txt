cmake_minimum_required(VERSION 3.10)
project(maxwell_vision)

# ---------- Compile options ----------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Ball message comes from maxwell_msgs (统一消息包)
find_package(maxwell_msgs REQUIRED)

# Image -> cv::Mat
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(cv_bridge REQUIRED)

find_package(Threads REQUIRED)

# CUDA is required (your code includes <cuda_runtime.h> unconditionally)
find_package(CUDAToolkit REQUIRED)

# Core headers (timer/config/common/singleton/math/parser/...)
# 你说这些都“保证存在”，因此这里用 REQUIRED，确保一遍编译就能发现缺失
find_package(maxwell_core REQUIRED)

# Optional: enable remote server streaming (requires tcp.hpp + libserver)
option(MAXWELL_WITH_SERVER "Enable remote server streaming in vision_core" OFF)

# ---------- Third-party / legacy libs ----------
# These are required for your current vision.cpp implementation (Darknet + imageproc + SL)
find_library(IMAGEPROC_LIB imageproc REQUIRED)
find_library(DARKNET_LIB darknet REQUIRED)
find_library(SL_LIB SL REQUIRED)

# Optional server lib
find_library(SERVER_LIB server)

# Try to locate include dirs for darknet/imageproc if they are not already exported by maxwell_core
find_path(DARKNET_INCLUDE_DIR darknet/network.h)
find_path(IMAGEPROC_INCLUDE_DIR imageproc/imageproc.hpp)

# ---------- Core library ----------
add_library(vision_core SHARED
  vision.cpp
)

# Compile in ROS2 mode (disables legacy updata/observer-based data injection)
target_compile_definitions(vision_core PUBLIC MAXWELL_ROS2)

if(MAXWELL_WITH_SERVER)
  target_compile_definitions(vision_core PUBLIC MAXWELL_WITH_SERVER)
endif()

target_include_directories(vision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(DARKNET_INCLUDE_DIR)
  target_include_directories(vision_core PUBLIC ${DARKNET_INCLUDE_DIR})
endif()
if(IMAGEPROC_INCLUDE_DIR)
  target_include_directories(vision_core PUBLIC ${IMAGEPROC_INCLUDE_DIR})
endif()

ament_target_dependencies(vision_core
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  maxwell_msgs
  maxwell_core
)

target_link_libraries(vision_core
  ${OpenCV_LIBRARIES}
  Threads::Threads
  CUDAToolkit::cudart
  ${IMAGEPROC_LIB}
  ${DARKNET_LIB}
  ${SL_LIB}
)

if(MAXWELL_WITH_SERVER AND SERVER_LIB)
  target_link_libraries(vision_core ${SERVER_LIB})
endif()

# rt: some platforms need it; safe on Linux
if(UNIX AND NOT APPLE)
  target_link_libraries(vision_core rt)
endif()

# ---------- Node executable ----------
add_executable(vision_node
  src/vision_node.cpp
)

ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  maxwell_msgs
)

target_link_libraries(vision_node
  vision_core
)

# ---------- Install ----------
install(TARGETS
  vision_core
  vision_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
