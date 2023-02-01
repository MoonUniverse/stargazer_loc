cmake_minimum_required(VERSION 2.8.3)
project(landmark_finder)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14 -pthread)
add_compile_options(-O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2
  dynamic_reconfigure
  message_generation
  std_msgs
  sensor_msgs
  image_transport
)
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_message_files(
    DIRECTORY msg
    FILES Landmark.msg LandmarkArray.msg LandmarkPoint.msg
    )
generate_messages(DEPENDENCIES std_msgs)


# Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
    cfg/LandmarkFinderInterface.cfg
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
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs std_msgs std_srvs tf tf2  cv_bridge image_transport
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
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(LandmarkFinder           src/LandmarkFinder.cpp)
target_link_libraries(LandmarkFinder        ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} yaml-cpp)

add_library(DebugVisualizer           src/DebugVisualizer.cpp)
target_link_libraries(DebugVisualizer        ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_library(LandmarkFinderInterfaceParameters           src/LandmarkFinderInterfaceParameters.cpp)
target_link_libraries(LandmarkFinderInterfaceParameters        ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES})

add_library(LandmarkFinderInterface           src/LandmarkFinderInterface.cpp)
target_link_libraries(LandmarkFinderInterface        ${catkin_LIBRARIES}  ${OpenCV_LIBRARIES} LandmarkFinderInterfaceParameters LandmarkFinder DebugVisualizer)

# Declare a C++ executable
add_executable(landmark_finder_node     src/landmark_finder_node.cpp)
target_link_libraries(landmark_finder_node  LandmarkFinderInterface )



