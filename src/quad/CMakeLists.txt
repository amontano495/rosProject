cmake_minimum_required(VERSION 2.8.3)
project(quad)
add_definitions(-std=c++11)

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

add_library( botLib botLib.cpp )

add_executable( quadLap quadLap.cpp )

add_executable( quadRWP quadRWP.cpp )

add_executable( quadWalk quadWalk.cpp botLib.cpp )

add_executable( quadDirection quadDirection.cpp botLib.cpp )

add_executable( radioListener radioListener.cpp )

target_link_libraries(quadLap ${catkin_LIBRARIES})

target_link_libraries(quadRWP ${catkin_LIBRARIES})

target_link_libraries(quadWalk botLib ${catkin_LIBRARIES})

target_link_libraries(quadDirection botLib ${catkin_LIBRARIES})

target_link_libraries(radioListener ${catkin_LIBRARIES})

install(TARGETS quadLap 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS quadRWP
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS quadWalk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS quadDirection
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS radioListener
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
