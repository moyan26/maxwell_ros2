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

# Ball message comes from maxwell_msgs
find_package(maxwell_msgs REQUIRED)

# Image -> cv::Mat
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(cv_bridge REQUIRED)

find_package(Threads REQUIRED)

# ---------- Runtime / CUDA switches ----------
# ON  : robot runtime build (needs imageproc/darknet/SL and usually CUDA)
# OFF : compile-only build on dev VM (no third-party vision libs)
# Default OFF so that anyone with ROS2 can build the workspace without custom libs.
option(MAXWELL_VISION_RUNTIME "Build full vision runtime pipeline" OFF)

# CUDA switch (only meaningful when runtime is ON)
option(MAXWELL_ENABLE_CUDA "Enable CUDA pipeline (robot runtime)" OFF)

# If not building runtime, force CUDA off so we never try to find/link CUDA
if(NOT MAXWELL_VISION_RUNTIME)
  set(MAXWELL_ENABLE_CUDA OFF CACHE BOOL "Enable CUDA pipeline" FORCE)
endif()

if(MAXWELL_ENABLE_CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()

# Core headers/libs (header-only or real)
find_package(maxwell_core REQUIRED)

# Optional: enable remote server streaming (requires tcp.hpp + libserver)
option(MAXWELL_WITH_SERVER "Enable remote server streaming in vision_core" OFF)

# ---------- Third-party / legacy libs (ONLY for runtime build) ----------
if(MAXWELL_VISION_RUNTIME)
  # These are required for your legacy vision.cpp implementation
  find_library(IMAGEPROC_LIB imageproc REQUIRED)
  find_library(DARKNET_LIB darknet REQUIRED)
  find_library(SL_LIB SL REQUIRED)

  # Optional server lib
  find_library(SERVER_LIB server)

  # Try to locate include dirs for darknet/imageproc if they are not already exported elsewhere
  find_path(DARKNET_INCLUDE_DIR darknet/network.h)
  find_path(IMAGEPROC_INCLUDE_DIR imageproc/imageproc.hpp)
endif()

# ---------- Core library ----------
add_library(vision_core SHARED
  vision.cpp
)

# Compile definitions
target_compile_definitions(vision_core PUBLIC
  MAXWELL_ROS2
  MAXWELL_VISION_RUNTIME=$<BOOL:${MAXWELL_VISION_RUNTIME}>
  MAXWELL_ENABLE_CUDA=$<BOOL:${MAXWELL_ENABLE_CUDA}>
)

if(MAXWELL_WITH_SERVER)
  target_compile_definitions(vision_core PUBLIC MAXWELL_WITH_SERVER)
endif()

target_include_directories(vision_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add third-party include dirs only when runtime build is enabled
if(MAXWELL_VISION_RUNTIME)
  if(DARKNET_INCLUDE_DIR)
    target_include_directories(vision_core PUBLIC ${DARKNET_INCLUDE_DIR})
  endif()
  if(IMAGEPROC_INCLUDE_DIR)
    target_include_directories(vision_core PUBLIC ${IMAGEPROC_INCLUDE_DIR})
  endif()
endif()

ament_target_dependencies(vision_core
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  maxwell_msgs
  maxwell_core
)

# Always link OpenCV + Threads
target_link_libraries(vision_core
  ${OpenCV_LIBRARIES}
  Threads::Threads
)

# Link legacy libs only in runtime mode
if(MAXWELL_VISION_RUNTIME)
  target_link_libraries(vision_core
    ${IMAGEPROC_LIB}
    ${DARKNET_LIB}
    ${SL_LIB}
  )

  if(MAXWELL_ENABLE_CUDA)
    target_link_libraries(vision_core CUDAToolkit::cudart)
  endif()

  if(MAXWELL_WITH_SERVER AND SERVER_LIB)
    target_link_libraries(vision_core ${SERVER_LIB})
  endif()
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