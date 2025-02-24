cmake_minimum_required(VERSION 3.8)
project(tf_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(tf_publisher_node src/tf_publisher_node.cpp)
ament_target_dependencies(tf_publisher_node
  rclcpp
  tf2_ros
  geometry_msgs
  sensor_msgs
)
target_link_libraries(tf_publisher_node yaml-cpp)

install(TARGETS
  tf_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 