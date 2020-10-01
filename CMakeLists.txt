cmake_minimum_required(VERSION 2.8.3)
project(romea_localisation_rtls)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

#######################
## Find dependencies ##
#######################
set(${PROJECT_NAME}_CATKIN_DEPS
  romea_rtls
  romea_localisation)

find_package(romea_filtering REQUIRED)
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )


find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

###########
## Build ##
###########
include_directories(
  include
  /usr/include/eigen3
  SYSTEM ${catkin_INCLUDE_DIRS}
#    SYSTEM ${EIGEN3_INCLUDE_DIRS},
  SYSTEM ${romea_filtering_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/rtls_localisation_plugin_diagnostics.cpp
  src/rtls_localisation_simple_trilateration.cpp
  src/rtls_localisation_transceivers.cpp
  src/rtls_localisation_ranges.cpp
  src/rtls_localisation_position_estimator.cpp
  src/rtls_localisation_pose_estimator.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif(CATKIN_ENABLE_TESTING)
