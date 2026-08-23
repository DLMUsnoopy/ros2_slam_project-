cmake_minimum_required(VERSION 3.16)
project(p03_slam_ros VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(p03_slam_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_conversions STATIC
  src/laser_scan_conversion.cpp
  src/odometry_conversion.cpp
  src/transform_conversion.cpp
)
target_compile_features(${PROJECT_NAME}_conversions PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_conversions
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_conversions
  p03_slam_core::p03_slam_core
)
ament_target_dependencies(${PROJECT_NAME}_conversions
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

add_library(${PROJECT_NAME}_node STATIC
  src/slam_node.cpp
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_conversions
)
ament_target_dependencies(${PROJECT_NAME}_node
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

add_executable(p03_slam_node
  src/slam_node_main.cpp
)
target_compile_features(p03_slam_node PUBLIC cxx_std_17)
target_link_libraries(p03_slam_node
  ${PROJECT_NAME}_node
)
ament_target_dependencies(p03_slam_node
  rclcpp
)

install(
  TARGETS
    ${PROJECT_NAME}_conversions
    ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}_conversions
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS p03_slam_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME}_conversions HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  p03_slam_core
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_laser_scan_conversion
    test/test_laser_scan_conversion.cpp
  )
  target_link_libraries(test_laser_scan_conversion
    ${PROJECT_NAME}_conversions
  )
  ament_target_dependencies(test_laser_scan_conversion
    sensor_msgs
  )

  ament_add_gtest(test_odometry_conversion
    test/test_odometry_conversion.cpp
  )
  target_link_libraries(test_odometry_conversion
    ${PROJECT_NAME}_conversions
  )
  ament_target_dependencies(test_odometry_conversion
    nav_msgs
  )

  ament_add_gtest(test_transform_conversion
    test/test_transform_conversion.cpp
  )
  target_link_libraries(test_transform_conversion
    ${PROJECT_NAME}_conversions
  )
  ament_target_dependencies(test_transform_conversion
    geometry_msgs
  )
endif()

ament_package()
