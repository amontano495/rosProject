cmake_minimum_required(VERSION 2.8.3)
project(quad)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
)

#
###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    roscpp
    mavros_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########


add_executable( quadLap quadLap.cpp )

target_link_libraries(quadLap ${catkin_LIBRARIES})


install(TARGETS quadLap 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
