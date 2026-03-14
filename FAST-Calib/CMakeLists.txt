cmake_minimum_required(VERSION 3.8)
project(fast_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)

set(DEPENDENCIES
  rclcpp
  sensor_msgs
  geometry_msgs
  rosbag2_cpp
  rosbag2_storage
  cv_bridge
  pcl_conversions
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_BUILD_TYPE Release)
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

add_executable(fast_calib src/main.cpp)
ament_target_dependencies(fast_calib ${DEPENDENCIES})
target_link_libraries(fast_calib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(multi_fast_calib src/multi_scene.cpp)
ament_target_dependencies(multi_fast_calib ${DEPENDENCIES})
target_link_libraries(multi_fast_calib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS fast_calib multi_fast_calib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/distance_filter_tool.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
