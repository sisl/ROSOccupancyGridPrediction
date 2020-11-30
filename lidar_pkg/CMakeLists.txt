cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 14)
project(lidar_pkg)


list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
list(APPEND CMAKE_PREFIX_PATH "/usr/local/libtorch")

find_package(catkin REQUIRED COMPONENTS
  lidar_msgs
  nodelet
  roscpp
  auto_messages
  lidar_msgs
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  roslib
  pcl_conversions
  message_filters
  tf2
  tf2_ros
  tf
)

## Setup include directories
include_directories(${catkin_INCLUDE_DIRS})


catkin_package(
#  INCLUDE_DIRS include
#   LIBRARIES nodelet_plugins
#   CATKIN_DEPENDS
#    lidar_msgs
#    roscpp
#    rospy
#    std_msgs
#  DEPENDS system_lib
)

## Create the nodelet tutorial library
add_library(nodelet_plugins
          src/aggregate_points.cpp
          src/aggregate_points.h
          src/mrf_ground_seg.cpp
          src/occupancy_grid_generation.cpp
          src/occupancy_grid_generation.h
	  #src/running_model.cpp
          src/inference.cpp
          src/inference.h
          src/inference_torch.cpp
          src/inference_torch.h
      	  src/utils.cpp
      	  src/utils.h
	  # src/frame_hack_nodelet.cpp
          src/visualize.cpp
          src/visualize.h
          )

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

target_link_libraries(nodelet_plugins "${TORCH_LIBRARIES}")
set_property(TARGET nodelet_plugins PROPERTY CXX_STANDARD 14)

#add_library(frame src/frame_hack_nodelet.cpp)


target_link_libraries(nodelet_plugins tensorflow_cc ${catkin_LIBRARIES} ${PROJECT_LIBRARIES} "${TORCH_LIBRARIES}")
#target_link_libraries(frame_hack_nodelet ${catkin_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(nodelet_plugins ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS nodelet_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES nodelet_plugins.xml mynodelet.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#add_executable(torch_test src/inference_torch.cpp)
#target_link_libraries(torch_test ${PROJECT_LIBRARIES} ${catkin_LIBRARIES} "${TORCH_LIBRARIES}")


#TENSORFLOW BITS

  # add modules
  list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules")

  # find packages
  find_package(TensorFlow REQUIRED)
  #find_package(Eigen REQUIRED)
  find_package(Protobuf REQUIRED)

  set(PROJECT_INCLUDE_DIRS ${TensorFlow_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})
  set(PROJECT_LIBRARIES ${TensorFlow_LIBRARIES} ${PROTOBUF_LIBRARIES})

  include_directories(${PROJECT_INCLUDE_DIRS})
  target_link_libraries(nodelet_plugins tensorflow_cc ${PROJECT_LIBRARIES} ${catkin_LIBRARIES})
