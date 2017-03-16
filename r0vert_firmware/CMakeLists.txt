cmake_minimum_required(VERSION 3.0)
project(r0vert_firmware)

# Must stay at the top for proper ordering of include directories

find_package(catkin REQUIRED COMPONENTS
    rosserial_arduino
    rosserial_msgs
    r0vert_msgs
    )

catkin_package(
    CATKIN_DEPENDS rosserial_arduino rosserial_msgs r0vert_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

# Custom target for generating ros_lib. Automatically executed during catkin_make.
add_custom_command(
    OUTPUT ${PROJECT_NAME}/ros_lib
    COMMAND rm -r ${PROJECT_SOURCE_DIR}/lib/ros_lib \;
    ${CATKIN_DEVEL_PREFIX}/env.sh rosrun rosserial_arduino make_libraries.py ${PROJECT_SOURCE_DIR}/lib
)
add_custom_target(${PROJECT_NAME}_ros_lib ALL # ALL to always execute
    DEPENDS ${PROJECT_NAME}/ros_lib rosserial_msgs_genpy)

# Dirty Hack! Only enable executable target when in clion (needs this line for code completion)
if (${CMAKE_ROOT} MATCHES ^.*clion.*$)
  include(CMakeListsPrivate.txt)
  add_executable(${PROJECT_NAME} ${INCLUDE_LIST} ${SRC_LIST})
endif ()
