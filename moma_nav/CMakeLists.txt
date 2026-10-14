cmake_minimum_required(VERSION 3.0.2)
project(moma_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(project_body_to_map_2d src/project_body_to_map_2d.cpp)
target_compile_options(project_body_to_map_2d PRIVATE -std=c++11)

target_link_libraries(project_body_to_map_2d
  ${catkin_LIBRARIES}
)
