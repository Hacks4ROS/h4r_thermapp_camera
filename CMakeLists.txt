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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(libthermappcamera SHARED src/ThermAppCam/thermapp/thermapp.c)

add_executable(thermapp_camera_node 
src/thermapp_camera_node.cpp 
src/ThermAppRos.cpp)

target_link_libraries(thermapp_camera_node libthermappcamera)

target_link_libraries(thermapp_camera_node
   ${LIBUSB1_LIBRARIES}
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS libthermappcamera thermapp_camera_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch udev
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


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
