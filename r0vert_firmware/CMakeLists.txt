cmake_minimum_required(VERSION 3.2)
project(r0vert_firmware)

# Must stay at the top for proper ordering of include directories
include(CMakeListsPrivate.txt)

find_package(catkin REQUIRED COMPONENTS
    rosserial_arduino
    rosserial_msgs
    )

catkin_package(
    CATKIN_DEPENDS rosserial_arduino rosserial_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_custom_target(
    PLATFORMIO_BUILD ALL
    COMMAND ${PLATFORMIO_CMD} -f -c clion run
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
    PLATFORMIO_UPLOAD ALL
    COMMAND ${PLATFORMIO_CMD} -f -c clion run --target upload
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
    PLATFORMIO_CLEAN ALL
    COMMAND ${PLATFORMIO_CMD} -f -c clion run --target clean
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
    PLATFORMIO_PROGRAM ALL
    COMMAND ${PLATFORMIO_CMD} -f -c clion run --target program
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
    PLATFORMIO_UPLOADFS ALL
    COMMAND ${PLATFORMIO_CMD} -f -c clion run --target uploadfs
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
    PLATFORMIO_UPDATE_ALL ALL
    COMMAND ${PLATFORMIO_CMD} -f -c clion update
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_custom_target(
    PLATFORMIO_REBUILD_PROJECT_INDEX ALL
    COMMAND ${PLATFORMIO_CMD} -f -c clion init --ide clion
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
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
  add_executable(${PROJECT_NAME} ${INCLUDE_LIST} ${SRC_LIST})
endif ()
