cmake_minimum_required(VERSION 2.8.3)
project(tbf_gripper_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
	roscpp
	shape_msgs
	geometric_shapes
    pcl_conversions
    pcl_ros
    cv_bridge
    message_generation
	)

find_package(Boost REQUIRED system)
find_package(OpenCV)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  CalcPointCloudTransform.srv
  GetCameraTransform.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES helperfunctions
 CATKIN_DEPENDS
    shape_msgs
    geometric_shapes
    geometry_msgs
    sensor_msgs
    cv_bridge
#  DEPENDS system_lib
)



add_compile_options(-std=c++11)
add_executable(mesh_importer src/mesh_importer.cpp)
add_library(helperfunctions src/helperfunctions.cpp)
target_link_libraries(helperfunctions ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(mesh_importer helperfunctions ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY python
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS helperfunctions
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
