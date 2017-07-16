cmake_minimum_required(VERSION 2.8.3)
project(m7_master)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  nav_msgs
  pauvsi_trajectory
)

find_package(Eigen3 REQUIRED)


catkin_package(
 	INCLUDE_DIRS include
#  LIBRARIES pauvsi_vio
  CATKIN_DEPENDS roscpp pauvsi_trajectory
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


add_library(polynomial_master include/m7_master/Polynomial.hpp)
set_target_properties(polynomial_master PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(polynomial_master ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

add_library(physics include/m7_master/Physics.h)
set_target_properties(physics PROPERTIES LINKER_LANGUAGE CXX)

add_library(roomba_callback include/m7_master/roombaCallback.hpp)
set_target_properties(roomba_callback  PROPERTIES LINKER_LANGUAGE CXX)


add_library(types_master include/m7_master/MasterTypes.h)
set_target_properties(types_master PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(types_master ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

add_library(NeuralNet include/m7_master/runNN.hpp)
set_target_properties(NeuralNet PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(NeuralNet types_master)


add_library(pid include/m7_master/PID.h)
set_target_properties(pid PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(pid ${Eigen_LIBRARIES} ${catkin_LIBRARIES})

add_executable(master src/m7_master.cpp)
target_link_libraries(master ${Eigen_LIBRARIES} ${catkin_LIBRARIES} roomba_callback NeuralNet polynomial_master types_master pid physics)

add_dependencies(master ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
