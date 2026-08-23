cmake_minimum_required(VERSION 3.16)
project(p03_slam_ros VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(p03_slam_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_conversions STATIC
  src/laser_scan_conversion.cpp
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
  sensor_msgs
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
  rclcpp
  sensor_msgs
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
ament_export_dependencies(p03_slam_core rclcpp sensor_msgs)

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
endif()

ament_package()
