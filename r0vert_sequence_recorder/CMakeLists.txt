cmake_minimum_required(VERSION 2.8.3)
project(r0vert_sequence_recorder)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    joy
    r0vert_msgs
    roscpp
    turtle_actionlib
    )

find_package(Boost 1.54 REQUIRED
    COMPONENTS system filesystem)
find_package(OpenCV 3 REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES sequence_recorder
    CATKIN_DEPENDS joy roscpp turtle_actionlib r0vert_msgs
    DEPENDS OpenCV Threads Boost GStreamer
)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    include
)

add_executable(sequence_recorder_node
    include/sequence_recorder/sequence.h
    src/sequence.cpp
    include/sequence_recorder/image_sequence_recorder.h
    src/image_sequence_recorder.cpp
    include/sequence_recorder/still_sequence_recorder.h
    src/still_sequence_recorder.cpp
    src/sequence_recorder_node.cpp
    )
add_dependencies(sequence_recorder_node r0vert_msgs_generate_messages_cpp)
target_link_libraries(sequence_recorder_node
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    opencv_core opencv_highgui opencv_imgproc opencv_calib3d
    )
