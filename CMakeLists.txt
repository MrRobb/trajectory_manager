###########################
## Package Configuration ##
###########################

cmake_minimum_required(VERSION 3.0.2)
project(trajectory_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include Conan-generated files
# include(${CMAKE_CURRENT_SOURCE_DIR}/build/Release/generators/conan_toolchain.cmake)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    message_generation
    actionlib_msgs
    tf2_ros
    # rapidcsv
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  AddCurrentLocation.srv
  GetCurrentLocation.srv
  GetRouteForObject.srv
  ClosestLocationToPosition.srv
  GetObjectPosition.srv
  GoTo.srv
  GoToLocation.srv
  LookAt.srv
  GraspPosition.srv
  SearchObject.srv
  Speak.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    trajectory_manager
    src/GensimModel.cpp
    src/Location.cpp
    src/LocationManager.cpp
    src/TrajectoryManager.cpp
)
target_link_libraries(trajectory_manager ${catkin_LIBRARIES})
add_dependencies(trajectory_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})