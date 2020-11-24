cmake_minimum_required(VERSION 2.8.3)
project(dhand_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

catkin_install_python(PROGRAMS src/driver.py src/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

