cmake_minimum_required(VERSION 3.0.2)
project(travesim_adapters)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES travesim_adapters
#  CATKIN_DEPENDS Boost roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/lib/data/team_command.cpp
  src/lib/data/entity_state.cpp
  src/lib/data/robot_state.cpp
  src/lib/data/field_state.cpp
  src/lib/vision_receiver.cpp
  src/lib/teams_sender.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context

add_executable(multi_sender_test
  src/bin/tests/multicast_sender.cpp
)

add_executable(multi_receiver_test
  src/bin/tests/multicast_receiver.cpp
)

add_executable(vision_adapter
  src/bin/vision_adapter.cpp
)

add_executable(teams_adapter
  src/bin/teams_adapter.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Specify libraries to link a library or executable target against

target_link_libraries(multi_sender_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(multi_receiver_test
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(vision_adapter
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(teams_adapter
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS vision_adapter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test src/lib/data/test/robot_state.test.cpp)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_travesim_adapters.cpp)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vision_adapter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
# endif()

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(vision_receiver.test test/vision_receiver.test src/lib/test/vision_receiver.test.cpp)
  target_link_libraries(vision_receiver.test ${catkin_LIBRARIES} ${PROJECT_NAME})

  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
