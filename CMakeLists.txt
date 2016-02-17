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
#  INCLUDE_DIRS include
#  LIBRARIES pips_trajectory_testing
  CATKIN_DEPENDS pips trajectory_generator
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

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native -std=c++11")


## Declare a cpp library
add_library(traj_gen_test
   src/GenAndTest.cpp
)

## Declare a cpp executable
add_executable(tester
   src/tester.cpp
)




## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pips_trajectory_testing_node pips_trajectory_testing_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(traj_gen_test
   ${catkin_LIBRARIES}
)

target_link_libraries(tester
   ${catkin_LIBRARIES} 
   traj_gen_test
)




