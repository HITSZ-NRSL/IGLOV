cmake_minimum_required(VERSION 2.8.3)

# set (pcl_DIR "/usr/local/include/pcl-1.11/pcl")
# set (octomap_DIR "/home/nrsl/MyApps/octomap-1.7.2/lib/cmake/octomap")

project(view_planning)
# set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++14)

find_package(octomap 1.7 REQUIRED)

# find_package(pcl 1.11 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  octomap
  octomap_msgs
  octomap_ros
  pcl_conversions
  sensor_msgs
  pcl_ros
  roscpp
  std_msgs
  tf
)

find_package(PCL REQUIRED QUIET COMPONENTS common sample_consensus io segmentation filters)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES view_planning
  CATKIN_DEPENDS octomap_msgs octomap_ros pcl_conversions pcl_ros roscpp std_msgs
  # DEPENDS octomap PCL
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIR}
  # ${PROJECT_SOURCE_DIR}/../../octomap_genarator_direct/include/octomap_genarator_direct
)

link_libraries(${OCTOMAP_LIBRARIES} )

AUX_SOURCE_DIRECTORY(src DIR_SRCS)

SET(TEST_MATH
${DIR_SRCS}
)

ADD_EXECUTABLE(${PROJECT_NAME}_node ${TEST_MATH})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
)