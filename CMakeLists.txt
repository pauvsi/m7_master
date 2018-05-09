cmake_minimum_required(VERSION 2.8.3)
project(mission7_master)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  nav_msgs
  multirotor_trajectory
)

find_package(Eigen3 REQUIRED)

generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
 	INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp multirotor_trajectory actionlib actionlib_msgs
#  DEPENDS system_lib
   DEPENDS eigen
   
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  	${Eigen_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)




add_library(master_types include/mission7_master/Types.h)
set_target_properties(master_types PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(master_types ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

add_library(m7master include/mission7_master/M7Master.cpp)
target_link_libraries(m7master ${Eigen_LIBRARIES} ${catkin_LIBRARIES} multirotor_trajectory_polynomial)

add_executable(master_node src/master_node.cpp)
target_link_libraries(master_node ${Eigen_LIBRARIES} ${catkin_LIBRARIES} m7master)



