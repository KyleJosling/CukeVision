cmake_minimum_required(VERSION 2.8.3)
project(cuke_vision)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  image_transport
  roscpp
  std_msgs
  message_generation
  tf
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
# NOTE : cv_bridge is causing linker issues. Refer here :https://answers.ros.org/question/312235/error-linking-to-opencv-dnn-library/ 
# anddd this one https://answers.ros.org/question/320609/how-to-linked-ros-with-opencv-345-dnn-module-error-undefined-reference-cvdnnexperimental_dnn_v2netnet/ 
)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(OpenCV 3.4.8 EXACT REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kinova_driver REQUIRED)

MESSAGE(WARNING "install path ${OpenCV_INSTALL_PATH}")
MESSAGE(WARNING "include dirs ${OpenCV_INCLUDE_DIRS} ")
MESSAGE(WARNING "library dirs ${OpenCV_LIBRARIES} ")
MESSAGE(WARNING "install path ${moveit_core_INSTALL_PATH}")
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################



# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
# None

################################################
## Find libraries
################################################
# None

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cuke_vision 
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
    moveit_core
    moveit_ros_planning_interface
  DEPENDS
    Boost
    OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${kinova_driver_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ardrone_object_tracking.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(cukeVisionNode src/cukeVisionNode.cpp)
 target_link_libraries(cukeVisionNode
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )
add_definitions(-std=c++11 -g -O3)
add_definitions(-DGUI)

add_executable(visionNode src/visionNode.cpp src/cukeDetector.cpp)
 target_link_libraries(visionNode
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
add_definitions(-std=c++11 -g -O3)
add_definitions(-DGUI)

add_executable(worldNode src/worldNode.cpp)
 target_link_libraries(worldNode
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${realsense2_LIBRARY}
   ${Eigen3_LIBRARY}
   ${kinova_driver_LIBRARIES}
 )
add_definitions(-std=c++11 -g -O3)
add_definitions(-DGUI)

add_executable(stepperControlNode src/stepperControlNode.cpp)
 target_link_libraries(stepperControlNode
   ${catkin_LIBRARIES}
 )
add_definitions(-std=c++11 -g -O3)
add_definitions(-DGUI)

add_executable(cutterControlNode src/cutterControlNode.cpp)
 target_link_libraries(cutterControlNode
   ${catkin_LIBRARIES}
 )
add_definitions(-std=c++11 -g -O3)
add_definitions(-DGUI)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ardrone_object_tracking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
