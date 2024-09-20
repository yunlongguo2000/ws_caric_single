cmake_minimum_required(VERSION 3.0.2)
project(octomap_gen)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  octomap_msgs
  octomap_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/octomap_gen.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
