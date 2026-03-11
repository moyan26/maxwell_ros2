cmake_minimum_required(VERSION 3.10)
project(maxwell_core)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only interface library
add_library(${PROJECT_NAME} STATIC
  src/parser.cpp
)
# 关键：静态库也用 -fPIC 编译，才能被链接进 libvision_core.so
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Headers are located directly under this folder and its subfolders (robot/, parser/, ...)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

# Install all headers recursively (robot/、parser/ 等子目录都会一起安装)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
  DESTINATION include
  FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "package.xml" EXCLUDE
)

# Export target so find_package(maxwell_core) works
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

ament_package()
