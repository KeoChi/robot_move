cmake_minimum_required(VERSION 2.8.3)
project(robot_move)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
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
#  INCLUDE_DIRS include
#  LIBRARIES robot_move
  CATKIN_DEPENDS geometry_msgs roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(goforward src/goforward.cpp)
target_link_libraries(goforward ${catkin_LIBRARIES})

add_executable(goincircles src/goincircles.cpp)
target_link_libraries(goincircles ${catkin_LIBRARIES})

add_executable(timed_out_and_back src/timed_out_and_back.cpp)
target_link_libraries(timed_out_and_back ${catkin_LIBRARIES})

add_executable(odom_out_and_back src/odom_out_and_back.cpp)
target_link_libraries(odom_out_and_back ${catkin_LIBRARIES})

add_executable(nav_square src/nav_square.cpp)
target_link_libraries(nav_square ${catkin_LIBRARIES})

