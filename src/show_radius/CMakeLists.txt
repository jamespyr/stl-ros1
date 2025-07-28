cmake_minimum_required(VERSION 3.0.2)
project(show_radius)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  pluginlib
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES show_radius
  CATKIN_DEPENDS roscpp rviz pluginlib sensor_msgs std_msgs pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)

add_library(show_radius src/point_radius_tool.cpp)
target_link_libraries(show_radius ${catkin_LIBRARIES} Qt5::Widgets ${PCL_LIBRARIES})

install(TARGETS show_radius LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})