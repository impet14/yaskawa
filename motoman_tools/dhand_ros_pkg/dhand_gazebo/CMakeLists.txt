cmake_minimum_required(VERSION 2.8.3)
project(dhand_gazebo)
find_package(catkin REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

###########
# Install #
###########
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

########
# Test #
########
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
