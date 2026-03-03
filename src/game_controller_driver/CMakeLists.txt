cmake_minimum_required(VERSION 3.8)
project(game_controller_driver)

# 使用 C++14
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ========== 创建可执行文件 ==========
add_executable(game_controller_node 
  src/game_controller_node.cpp
  src/game_controller_main.cpp
)

ament_target_dependencies(game_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
)

target_include_directories(game_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ========== 监听节点（调试用）==========
add_executable(game_controller_listener
  src/game_controller_listener.cpp
)

ament_target_dependencies(game_controller_listener
  rclcpp
  std_msgs
)

# ========== 模拟发送器（测试用）==========
add_executable(game_controller_mock
  src/game_controller_mock.cpp
)

ament_target_dependencies(game_controller_mock
  rclcpp
  std_msgs
)

# ========== 安装 ==========
install(TARGETS game_controller_node game_controller_listener game_controller_mock
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
