cmake_minimum_required(VERSION 3.0.2)
project(gimbal_controller)

# set (CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++14)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planning
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/gimbal_controller
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

AUX_SOURCE_DIRECTORY(src DIR_SRCS)

SET(TEST_MATH
${DIR_SRCS}
)

ADD_EXECUTABLE(${PROJECT_NAME}_node ${TEST_MATH})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)