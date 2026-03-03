cmake_minimum_required(VERSION 3.8)
project(camera_driver)

# 使用 C++14
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# ========== 创建可执行文件 ==========
add_executable(camera_node 
  src/camera_node.cpp
  src/camera_driver_main.cpp
)

# 链接库
target_link_libraries(camera_node 
  ${OpenCV_LIBS}
)

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_include_directories(camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# ========== 监听节点（调试用）==========
add_executable(camera_listener
  src/camera_listener.cpp
)

target_link_libraries(camera_listener
  ${OpenCV_LIBS}
)

ament_target_dependencies(camera_listener
  rclcpp
  sensor_msgs
  cv_bridge
)

# ========== 安装 ==========
install(TARGETS camera_node camera_listener
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装 config 文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
