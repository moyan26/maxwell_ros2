cmake_minimum_required(VERSION 3.16)
project(maxwell_world)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(maxwell_msgs REQUIRED)

add_subdirectory(player/core)

add_executable(worldmodel_node
  src/worldmodel_node.cpp
)

target_include_directories(worldmodel_node
  PRIVATE
    ${tf2_INCLUDE_DIRS}
)

target_link_libraries(worldmodel_node
  PRIVATE
    worldmodel_core
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${maxwell_msgs_TARGETS}
    ${tf2_LIBRARIES}
)

install(TARGETS worldmodel_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
