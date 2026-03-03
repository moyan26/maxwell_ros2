cmake_minimum_required(VERSION 3.8)
project(maxwell_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================= Dependencies =================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(maxwell_msgs REQUIRED)
find_package(maxwell_vision REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ================= Executable =================

add_executable(world_node
  src/world_node.cpp
)

ament_target_dependencies(world_node
  rclcpp
  maxwell_msgs
  maxwell_vision
  sensor_msgs
  geometry_msgs
)

# ⭐ 
target_include_directories(world_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ================= Install =================

install(TARGETS
  world_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
