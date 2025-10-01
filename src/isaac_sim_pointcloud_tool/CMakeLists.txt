cmake_minimum_required(VERSION 3.5)
project(isaac_sim_pointcloud_tool)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(converter src/converter.cpp)

ament_target_dependencies(converter
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_msgs
)

target_link_libraries(converter
  ${PCL_LIBRARIES}
)

install(TARGETS
  converter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
