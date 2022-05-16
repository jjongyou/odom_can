cmake_minimum_required(VERSION 3.8)
project(odom_can)

# print include directory path
# message($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
# message($<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(can_odometry STATIC src/can_odometry.cpp)
target_compile_features(can_odometry PRIVATE cxx_std_17)
ament_target_dependencies(can_odometry rclcpp
  std_msgs geometry_msgs)
target_include_directories(can_odometry
  PUBLIC
  # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

add_executable(odom_node src/odom_node.cpp)
ament_target_dependencies(odom_node rclcpp)
target_link_libraries(odom_node can_odometry)

install(TARGETS
  odom_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
