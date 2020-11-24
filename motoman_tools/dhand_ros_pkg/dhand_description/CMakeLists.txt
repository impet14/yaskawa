cmake_minimum_required(VERSION 2.8.3)
project(dhand_description)
find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
)

###########
# Install #
###########
set(LAUNCH_DIRS
  launch
  launch/rviz
)
set(MESH_DIRS
  meshes/collision
  meshes/collision/finger
  meshes/visual/adapter
  meshes/visual/base
  meshes/visual/finger/base
  meshes/visual/finger/middle
  meshes/visual/finger/top
)
set(URDF_DIRS
  robots
  urdf
  urdf/adapter
  urdf/base
  urdf/finger
  )

foreach(dir ${LAUNCH_DIRS} ${MESH_DIRS} ${URDF_DIRS})
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
