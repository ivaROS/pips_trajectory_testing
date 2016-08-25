cmake_minimum_required(VERSION 2.8.3)
project(pips_trajectory_testing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  image_view
  tf
  tf2
  
  trajectory_generator
  pips
  turtlebot_trajectory_controller
  )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



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
  INCLUDE_DIRS include
##  LIBRARIES pips_trajectory_testing
  CATKIN_DEPENDS pips trajectory_generator turtlebot_trajectory_controller
##  DEPENDS system_lib
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


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -std=c++11 ${OpenMP_CXX_FLAGS}")

## Declare a cpp library
add_library(pips_trajectory_tester
   src/pips_trajectory_tester.cpp
)


## Declare a cpp library
add_library(pips_demo
   src/pips_demo.cpp
)

## Declare a cpp executable
add_executable(pips_demo_node
   src/pips_demo_node.cpp
)

## Declare a cpp library
add_library(pips_demo_nodelet MODULE
   src/pips_demo_nodelet.cpp
)

## Declare a cpp library
add_library(pips_controller
   src/pips_controller.cpp
)

## Declare a cpp library
add_library(pips_controller_nodelet MODULE
   src/pips_controller_nodelet.cpp
)

## Declare standalone pips trajectory controller
add_executable(pips_controller_node
   src/pips_controller_node.cpp
)

add_executable(delayed_nodelet 
   src/delayed_nodelet.cpp
)

add_library(rosbag_publisher_nodelet MODULE
   src/rosbag_nodelet_publisher.cpp
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pips_trajectory_testing_node pips_trajectory_testing_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pips_trajectory_tester
   ${catkin_LIBRARIES}
)

target_link_libraries(pips_demo
   ${catkin_LIBRARIES} 
   pips_trajectory_tester
)

target_link_libraries(pips_demo_nodelet
   ${catkin_LIBRARIES} 
   pips_demo
)

target_link_libraries(pips_demo_node
   ${catkin_LIBRARIES} 
   pips_demo
)


target_link_libraries(pips_controller
   ${catkin_LIBRARIES} 
   pips_trajectory_tester
)

target_link_libraries(pips_controller_nodelet
   ${catkin_LIBRARIES} 
   pips_controller
)

target_link_libraries(pips_controller_node
   ${catkin_LIBRARIES} 
   pips_controller
)

target_link_libraries(delayed_nodelet
   ${catkin_LIBRARIES}
)

target_link_libraries(rosbag_publisher_nodelet
   ${catkin_LIBRARIES}
)

