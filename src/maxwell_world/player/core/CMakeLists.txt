# player/core/CMakeLists.txt
#
# ROS2迁移建议：把 worldmodel 等核心逻辑编成可复用库，供 worldmodel_node 链接。
# 这里保持为“纯 CMake 子目录”，由上层 ROS2 包 CMakeLists.txt add_subdirectory() 调用。

add_library(core STATIC)
add_library(worldmodel_core ALIAS core)

target_sources(core PRIVATE
  worldmodel.cpp
)

set_target_properties(core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(core PUBLIC cxx_std_17)

target_include_directories(core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/..>
)

find_package(Threads REQUIRED)
target_link_libraries(core PUBLIC Threads::Threads)

# Eigen (optional)
if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 QUIET)
endif()
if(TARGET Eigen3::Eigen)
  target_link_libraries(core PUBLIC Eigen3::Eigen)
endif()

# If you really want legacy Sensor/Observer adapter, build with:
#   target_compile_definitions(core PUBLIC MAXWELL_LEGACY_SENSORS)
