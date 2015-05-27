cmake_minimum_required(VERSION 2.8.3)
project(pr2_3dnav_visualization)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread date_time system filesystem)

find_package(PkgConfig REQUIRED)

# Ogre
pkg_check_modules(OGRE OGRE)
link_directories( ${OGRE_LIBRARY_DIRS} )

# Qt Stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  moveit_ros_warehouse
  moveit_ros_perception
  cmake_modules
  eigen_conversions
  geometric_shapes
  interactive_markers
  class_loader
  rviz
  tf
  roscpp
  rosconsole
  object_recognition_msgs
)
find_package(Eigen REQUIRED)
catkin_python_setup()
catkin_package(
  LIBRARIES
    pr2_3dnav_slice_rviz_plugin_core
  INCLUDE_DIRS
    pr2_3dnav_slice_rviz_plugin/include
  CATKIN_DEPENDS
    moveit_ros_planning_interface
    moveit_ros_robot_interaction
    )

  include_directories(pr2_3dnav_slice_rviz_plugin/include
		    ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${QT_INCLUDE_DIR}
                    ${OGRE_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(pr2_3dnav_slice_rviz_plugin)

install(FILES
  pr2_3dnav_slice_rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
