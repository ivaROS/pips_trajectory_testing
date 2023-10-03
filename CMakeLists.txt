cmake_minimum_required(VERSION 2.8.3)
project(pips_trajectory_testing)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  visualization_msgs
  image_transport
  tf
  tf2
  tf2_ros
  dynamic_reconfigure
  trajectory_generator
  pips
  tf2_pips
  message_generation
  std_msgs
  geometry_msgs
  )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
# find_package(OpenMP)

generate_dynamic_reconfigure_options(
  cfg/PipsController.cfg
  cfg/PipsTrajectoryTester.cfg
  )

add_service_files(
  FILES
  TestCollisionStamped.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
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
  LIBRARIES  pips_cc_wrapper depth_image_cc_wrapper #OpenMP::OpenMP_CXX #pips_trajectory_tester
  CATKIN_DEPENDS pips trajectory_generator dynamic_reconfigure roscpp sensor_msgs image_transport tf tf2 tf2_ros tf2_pips message_runtime std_msgs geometry_msgs
  DEPENDS Boost
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


add_library(pips_cc_wrapper
   src/pips_cc_wrapper.cpp
)

target_compile_options(pips_cc_wrapper PUBLIC  -std=c++14)

add_dependencies(pips_cc_wrapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pips_cc_wrapper
  PUBLIC
   ${catkin_LIBRARIES}
)


add_library(depth_image_cc_wrapper
   src/depth_image_cc_wrapper.cpp
)

target_compile_options(depth_image_cc_wrapper PUBLIC  -std=c++14)

#add_dependencies(pips_cc_wrapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(depth_image_cc_wrapper
  PUBLIC
  pips_cc_wrapper
   ${catkin_LIBRARIES}
)


add_executable(depth_image_cc_node
  src/depth_image_cc_node.cpp
)

target_compile_options(depth_image_cc_node PUBLIC  -std=c++14)

#add_dependencies(depth_image_cc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(depth_image_cc_node
  PUBLIC
  depth_image_cc_wrapper
   ${catkin_LIBRARIES}
)
# 
# if(NOT TARGET OpenMP::OpenMP_CXX)
#     find_package(Threads REQUIRED)
#     add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
#     set_property(TARGET OpenMP::OpenMP_CXX
#                  PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
#     # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
#     set_property(TARGET OpenMP::OpenMP_CXX
#                  PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
# 
# endif()

# TODO: Add interface library for trajectory testing to ensure that OMP flags get applied to dependent targets: http://mariobadr.com/creating-a-header-only-library-with-cmake.html
# NOTE: The above may not actually be useful, since I can't figure out how to get a dependent target to use the compile flags

# ## Declare a cpp library
# add_library(pips_trajectory_tester src/pips_trajectory_tester.cpp)
# #target_sources(pips_trajectory_tester PUBLIC "pips_trajectory_tester.h")
# target_include_directories(pips_trajectory_tester PUBLIC include/)
# target_compile_options(pips_trajectory_tester PUBLIC -std=c++11 ${OpenMP_CXX_FLAGS})
# target_link_libraries(pips_trajectory_tester
#   PUBLIC
#   ${OpenMP_CXX_FLAGS}
#    ${catkin_LIBRARIES}
# )
 
# 
# add_dependencies(pips_trajectory_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
# target_compile_options(pips_trajectory_tester PRIVATE -Wall ${OpenMP_CXX_FLAGS})
# target_compile_options(pips_trajectory_tester PUBLIC  -std=c++11)
# 
# ## Declare a cpp library
# add_library(obstacle_avoidance_controller
#    src/obstacle_avoidance_controller.cpp
# )
# 
# add_dependencies(obstacle_avoidance_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 
# ## Declare a cpp library
# add_library(pips_controller
#    src/pips_controller.cpp
# )
# 
# ## Declare a cpp library
# add_library(pips_controller_nodelet MODULE
#    src/pips_controller_nodelet.cpp
# )
# 
# ## Declare standalone pips trajectory controller
# add_executable(pips_controller_node
#    src/pips_controller_node.cpp
# )
# 
# add_library(rosbag_publisher_nodelet MODULE
#    src/rosbag_nodelet_publisher.cpp
# )
# 
# 
# ## Specify libraries to link a library or executable target against
# target_link_libraries(pips_trajectory_tester
#   PUBLIC
#    ${catkin_LIBRARIES}
#    PRIVATE
#    ${OpenMP_CXX_FLAGS}
# )
# 
# target_link_libraries(obstacle_avoidance_controller
#    ${catkin_LIBRARIES} 
#    pips_trajectory_tester
# )
# 
# target_link_libraries(pips_controller
#     obstacle_avoidance_controller
#    depth_image_cc_wrapper
#    ${catkin_LIBRARIES} 
# )
# 
# target_link_libraries(pips_controller_nodelet
#    ${catkin_LIBRARIES} 
#    pips_controller
# )
# 
# target_link_libraries(pips_controller_node
#    ${catkin_LIBRARIES} 
#    pips_controller
# )
# 
# target_link_libraries(rosbag_publisher_nodelet
#    ${catkin_LIBRARIES}
# )


