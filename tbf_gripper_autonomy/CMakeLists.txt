cmake_minimum_required(VERSION 2.8.3)
project(tbf_gripper_autonomy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    geometry_msgs
    sensor_msgs
    haf_grasping
    ur_kinematics
)
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES tbf_gripper_autonomy
  CATKIN_DEPENDS
    ur_kinematics
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(test_ik src/test/test_ik.cpp)
target_link_libraries(test_ik ${catkin_LIBRARIES})
add_dependencies(test_ik ur_kinematics)

catkin_install_python(PROGRAMS scripts/run_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
