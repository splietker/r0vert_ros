cmake_minimum_required(VERSION 2.8.3)
project(r0vert_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  BatteryVoltage.msg
  WheelVelocity.msg
)

generate_messages(
  DEPENDENCIES
)
catkin_package(CATKIN_DEPENDS message_runtime)
