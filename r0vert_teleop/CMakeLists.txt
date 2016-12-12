cmake_minimum_required(VERSION 2.8.3)
project(r0vert_teleop)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
  turtle_actionlib
)

catkin_package(
  CATKIN_DEPENDS joy roscpp turtle_actionlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_joy src/teleop_joy.cpp)
target_link_libraries(teleop_joy ${catkin_LIBRARIES})