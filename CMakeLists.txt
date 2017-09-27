cmake_minimum_required(VERSION 2.8.3)
project(pips_trajectory_testing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
#  image_view
  tf
  tf2
  dynamic_reconfigure
  
  trajectory_generator
  pips
  turtlebot_trajectory_controller
  )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenMP)

generate_dynamic_reconfigure_options(
  cfg/PipsController.cfg
  cfg/PipsTrajectoryTester.cfg
  )


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
  LIBRARIES obstacle_avoidance_controller pips_trajectory_tester pips_controller
  CATKIN_DEPENDS pips trajectory_generator turtlebot_trajectory_controller dynamic_reconfigure roscpp sensor_msgs image_transport tf tf2
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


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall ")



## Declare a cpp library
add_library(pips_trajectory_tester
   src/pips_trajectory_tester.cpp
)

add_dependencies(pips_trajectory_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_compile_options(pips_trajectory_tester PRIVATE -Wall ${OpenMP_CXX_FLAGS})
target_compile_options(pips_trajectory_tester PUBLIC  -std=c++11)


##Don't need the next 3 for now

## Declare a cpp library
#add_library(pips_demo
#   src/pips_demo.cpp
#)

## Declare a cpp executable
#add_executable(pips_demo_node
#   src/pips_demo_node.cpp
#)

## Declare a cpp library
#add_library(pips_demo_nodelet MODULE
#   src/pips_demo_nodelet.cpp
#)

## Declare a cpp library
add_library(obstacle_avoidance_controller
   src/obstacle_avoidance_controller.cpp
)

add_dependencies(obstacle_avoidance_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

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

#add_executable(delayed_nodelet 
#   src/delayed_nodelet.cpp
#)

add_library(rosbag_publisher_nodelet MODULE
   src/rosbag_nodelet_publisher.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(pips_trajectory_tester
  PUBLIC
   ${catkin_LIBRARIES}
   PRIVATE
   ${OpenMP_CXX_FLAGS}
)


#target_link_libraries(pips_demo
#   ${catkin_LIBRARIES} 
#   pips_trajectory_tester
#)

#target_link_libraries(pips_demo_nodelet
#   ${catkin_LIBRARIES} 
#   pips_demo
#)

#target_link_libraries(pips_demo_node
#   ${catkin_LIBRARIES} 
#   pips_demo
#)


target_link_libraries(obstacle_avoidance_controller
   ${catkin_LIBRARIES} 
   pips_trajectory_tester
)

target_link_libraries(pips_controller
   ${catkin_LIBRARIES} 
   obstacle_avoidance_controller
)

target_link_libraries(pips_controller_nodelet
   ${catkin_LIBRARIES} 
   pips_controller
)

target_link_libraries(pips_controller_node
   ${catkin_LIBRARIES} 
   pips_controller
)

#target_link_libraries(delayed_nodelet
#   ${catkin_LIBRARIES}
#)

target_link_libraries(rosbag_publisher_nodelet
   ${catkin_LIBRARIES}
)

