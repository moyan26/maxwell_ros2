cmake_minimum_required(VERSION 3.8)
project(imu_driver)

# 使用 C++14
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

# 查找 Boost
find_package(Boost REQUIRED COMPONENTS system thread)

# 包含目录
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# ========== 创建可执行文件 ==========
add_executable(imu_node 
  src/imu_node.cpp
  src/imu_driver_main.cpp
)

target_link_libraries(imu_node 
  ${Boost_LIBRARIES}
)

ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  Eigen3
)

# ========== 创建库（供其他节点使用）==========
add_library(${PROJECT_NAME} SHARED
  src/imu_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  Eigen3
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ========== 安装 ==========
install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ========== 测试 ==========
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ========== 订阅者示例（可选编译）==========
add_executable(imu_subscriber_example
  src/imu_subscriber_example.cpp
)

ament_target_dependencies(imu_subscriber_example
  rclcpp
  sensor_msgs
  std_msgs
  tf2
)

install(TARGETS imu_subscriber_example
  DESTINATION lib/${PROJECT_NAME}
)

# ========== IMU 监听器（调试用）==========
add_executable(imu_listener
  src/imu_listener.cpp
)

ament_target_dependencies(imu_listener
  rclcpp
  sensor_msgs
)

install(TARGETS imu_listener
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
