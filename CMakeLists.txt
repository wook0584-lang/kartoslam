cmake_minimum_required(VERSION 3.8)
project(slam_karto)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

add_library(correlation_scan_match SHARED
  src/correlation_scan_match.cpp
)
target_include_directories(correlation_scan_match
  PUBLIC
    include
)
target_link_libraries(correlation_scan_match ${Boost_LIBRARIES})

add_executable(slam_karto src/slam_karto.cpp)
target_include_directories(slam_karto
  PRIVATE
    include
)
ament_target_dependencies(slam_karto
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(slam_karto
  correlation_scan_match
  ${Boost_LIBRARIES}
)

install(TARGETS correlation_scan_match
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS slam_karto
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(correlation_scan_match)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
