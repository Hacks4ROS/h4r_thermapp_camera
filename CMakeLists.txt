cmake_minimum_required(VERSION 2.8.3)
project(h4r_thermapp_camera)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
  cv_bridge
)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

#Libusb
find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})

#Generate dynamic reconfigure stuff
generate_dynamic_reconfigure_options(cfg/h4r_thermapp_camera_node.cfg)

catkin_package(
   INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(libthermappcamera SHARED src/ThermAppCam/thermapp/thermapp.c)

## Declare a cpp executable
add_executable(thermapp_camera_node 
src/thermapp_camera_node.cpp 
src/ThermAppRos.cpp)

target_link_libraries(thermapp_camera_node libthermappcamera)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(thermapp_camera_node thermapp_camera_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(thermapp_camera_node
   ${LIBUSB1_LIBRARIES}
   ${catkin_LIBRARIES}
)

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
# install(TARGETS thermapp_camera thermapp_camera_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_thermapp_camera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
