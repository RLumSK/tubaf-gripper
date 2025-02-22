cmake_minimum_required(VERSION 2.8.3)
project(tbf_gripper_autonomy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    eigen_conversions
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    cmake_modules
    geometric_shapes
    moveit_msgs
    message_generation
)

find_package(Boost REQUIRED system)

find_package(Eigen3 REQUIRED)

catkin_python_setup()

add_service_files(
  FILES
  GenerateSetPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  object_recognition_msgs
  visualization_msgs
  geometry_msgs
)

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
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    ur_kinematics
#  DEPENDS system_lib
)



add_compile_options(-std=c++11)
include_directories(include ${catkin_INCLUDE_DIRS})
#add_library(planningwrapper src/planningwrapper.cpp)
add_executable(planning src/planning.cpp src/planningwrapper.cpp)
target_link_libraries(planning ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
