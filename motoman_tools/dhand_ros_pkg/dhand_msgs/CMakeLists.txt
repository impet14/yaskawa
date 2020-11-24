cmake_minimum_required(VERSION 2.8.3)
project(dhand_msgs)
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Servo_move.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
