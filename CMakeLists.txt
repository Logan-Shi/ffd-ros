cmake_minimum_required(VERSION 2.8.3)
project(fast_frontier_detector)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	message_generation
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES g2o2occupancy_ros
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)



set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11")




##FIND_PACKAGE(SuiteSparse REQUIRED)
##include_directories(${CSPARSE_INCLUDE_DIR})

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
## Declare a C++ executable

add_executable(gfs-carmen
  src/gfs-carmen.cpp
  ## include/utils/commandline.h
  ## include/gridfastslam/gridslamprocessor.h
  ## include/gridfastslam/gridslamprocessor.cpp
  ## include/particlefilter/particlefilter.h
  ## include/particlefilter/particlefilter.cpp
  ## include/log/sensorlog.h
  ## include/log/sensorlog.cpp
  ## include/sensor/sensor_base/sensorreading.h
  ## include/sensor/sensor_base/sensorreading.cpp
  include/scanmatcher/FrontierIndexer.h
  include/scanmatcher/FrontierIndexer.cpp
  ## include/grid/map.h
  include/scanmatcher/FrontierManager.h
  include/scanmatcher/FrontierManager.cpp
  ## include/configfile/configfile.h
  ## include/configfile/configfile.cpp
  ## include/sensor/sensor_odometry/odometrysensor.h
  ## include/sensor/sensor_odometry/odometrysensor.cpp
  ## include/sensor/sensor_range/rangesensor.h
  ## include/sensor/sensor_range/rangesensor.cpp
)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(gfs-carmen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(gfs-carmen
  ${catkin_LIBRARIES} 
)


