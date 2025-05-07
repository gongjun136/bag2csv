cmake_minimum_required(VERSION 3.0.2)
project(rosbag_to_csv)


add_subdirectory(livox_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  sensor_msgs
  nav_msgs
  message_filters
  pcl_conversions
  pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/build/devel/include
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(bag_to_csv bag_to_csv.cpp)
target_link_libraries(bag_to_csv ${catkin_LIBRARIES})

add_executable(read_pcd read_pcd.cpp)
target_link_libraries(read_pcd  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(read_csv read_csv.cpp)
target_link_libraries(read_csv  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

