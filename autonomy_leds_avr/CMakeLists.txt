cmake_minimum_required(VERSION 2.8.3)
project(autonomy_leds_avr)

find_package(catkin REQUIRED COMPONENTS
  rosserial_client
  rosserial_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

rosserial_generate_ros_lib(
  PACKAGE rosserial_client
  SCRIPT make_libraries
)

include(ExternalProject)
ExternalProject_Add(autonomy_leds_firmware
  URL ${CMAKE_CURRENT_SOURCE_DIR}/firmware
  CONFIGURE_COMMAND ""
  # Unfortunately the Makfile does not handle absolute paths to source files correctly,
  # since the OBJDIR treats everything as relative. The following is a nasty hack
  #BUILD_COMMAND make install ROSLIBDIR=${${PROJECT_NAME}_ROS_LIB_DIR} DESTDIR=${CATKIN_DEVEL_PREFIX}/share
  BUILD_COMMAND make install ROSLIBDIR=../../../ros_lib DESTDIR=${CATKIN_DEVEL_PREFIX}/share
  INSTALL_COMMAND 
  BUILD_IN_SOURCE 1
)

add_dependencies(autonomy_leds_firmware autonomy_leds_avr_ros_lib)

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
# install(TARGETS autonomy_leds autonomy_leds_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_autonomy_leds.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
