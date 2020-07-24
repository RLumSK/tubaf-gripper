cmake_minimum_required(VERSION 2.8.3)
project(tbf_gripper_perception)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   shape_msgs
   std_msgs
   message_generation
   geometry_msgs
   roscpp
   pcl_conversions
   pcl_ros
   sensor_msgs
   tbf_gripper_tools
   dynamic_reconfigure
   object_recognition_msgs
   )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DetectedObject.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   TogglePerception.srv
   ObjectClassifierService.srv
   IdentifyFloor.srv
#  Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   shape_msgs
   geometry_msgs
   object_recognition_msgs
   # Or other packages containing msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  config/surface_matching.cfg
  config/icp_matching.cfg
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
 INCLUDE_DIRS 
  include
 LIBRARIES 
  ${PROJECT_NAME}
  helperfunctions
 CATKIN_DEPENDS
  shape_msgs
  message_runtime
  std_msgs
  geometric_shapes
  cv_bridge
 DEPENDS 
  opencv
)

###########
## Build ##
###########

catkin_install_python(PROGRAMS scripts/pl_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

add_compile_options(-std=c++11)
add_executable(surface_matching src/surface_matching.cpp)
add_executable(icp_matching src/icp_matching.cpp)
add_library(objectsearch src/drostobjectsearch.cpp src/cvicpobjectsearch.cpp src/pclicpobjectsearch.cpp src/icpobjectsearch.cpp src/objectsearch.cpp)
add_dependencies(objectsearch ${PROJECT_NAME}_gencfg)
target_link_libraries(objectsearch ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES}  )
target_link_libraries(surface_matching objectsearch ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES}  )
target_link_libraries(icp_matching objectsearch ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES}  )

install(TARGETS surface_matching
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
