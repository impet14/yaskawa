cmake_minimum_required(VERSION 2.8.3)
project(dhand_control)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  roscpp
  sensor_msgs
  )

catkin_package(CATKIN_DEPENDS
  roscpp
  sensor_msgs
  )
include_directories(
  ${catkin_INCLUDE_DIRS}
  )

#########
# Build #
#########
add_executable(dhand_joint_state_republisher src/dhand_joint_state_republisher.cpp)
target_link_libraries(dhand_joint_state_republisher
  ${catkin_LIBRARIES}
  )

###########
# Install #
###########
install(TARGETS dhand_joint_state_republisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
foreach(dir config launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

########
# Test #
########
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()


