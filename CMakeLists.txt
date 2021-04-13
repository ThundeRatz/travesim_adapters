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

## Add subdirectories
add_subdirectory(proto)

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
#  INCLUDE_DIRS include
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
add_library(${PROJECT_NAME}_core
  src/lib/data/team_command.cpp
  src/lib/data/entity_state.cpp
  src/lib/data/robot_state.cpp
  src/lib/data/field_state.cpp
)

add_library(${PROJECT_NAME}_udp_comm
  src/lib/udp/multicast_receiver.cpp
  src/lib/udp/multicast_sender.cpp
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context

add_executable(multi_sender_example
  examples/udp/multicast_sender_example.cpp
)

add_executable(multi_receiver_example
  examples/udp/multicast_receiver_example.cpp
)

## Specify libraries to link a library or executable target against

target_link_libraries(multi_sender_example
  ${PROJECT_NAME}_udp_comm
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(multi_receiver_example
  ${PROJECT_NAME}_udp_comm
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_travesim_adapters.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
