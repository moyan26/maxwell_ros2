cmake_minimum_required(VERSION 3.8)
project(motor_driver)

# 使用 C++14
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# ========== 创建可执行文件 ==========
add_executable(motor_node 
  src/motor_node.cpp
  src/motor_main.cpp
)

ament_target_dependencies(motor_node
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)

target_include_directories(motor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ========== 监听节点（调试用）==========
add_executable(motor_listener
  src/motor_listener.cpp
)

ament_target_dependencies(motor_listener
  rclcpp
  sensor_msgs
)

# ========== 关节状态查看器（格式化显示）==========
add_executable(joint_state_viewer
  src/joint_state_viewer.cpp
)

ament_target_dependencies(joint_state_viewer
  rclcpp
  sensor_msgs
)

# ========== 安装 ==========
install(TARGETS motor_node motor_listener joint_state_viewer
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
