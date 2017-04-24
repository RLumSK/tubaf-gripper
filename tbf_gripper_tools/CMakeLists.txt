cmake_minimum_required(VERSION 2.8.3)
project(tbf_gripper_tools)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	shape_msgs
	geometric_shapes
	)

find_package(Boost REQUIRED system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tbf_gripper_autonomy
  CATKIN_DEPENDS
    shape_msgs
    geometric_shapes
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(mesh_importer src/mesh_importer.cpp)
target_link_libraries(mesh_importer ${catkin_LIBRARIES} ${Boost_LIBRARIES})
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
