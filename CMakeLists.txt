cmake_minimum_required(VERSION 3.8)
project(laser_segmentation)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################
## Find  dependencies                         ##
################################################
## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(slg_msgs REQUIRED) # formerly simple_laser_geometry 

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${slg_msgs_INCLUDE_DIRS}
)

# executable node
add_executable (${PROJECT_NAME}
  src/segmentation/segmentationJumpDistance.cpp
  src/segmentation/segmentationJumpDistanceMerge.cpp
  src/laserSegmentation.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs 
  sensor_msgs
  visualization_msgs
  slg_msgs
  nav2_util
)
target_link_libraries(${PROJECT_NAME})


#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
