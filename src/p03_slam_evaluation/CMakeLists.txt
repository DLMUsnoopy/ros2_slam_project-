cmake_minimum_required(VERSION 3.16)
project(p03_slam_evaluation VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(p03_slam_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME}_core STATIC
  src/trajectory_evaluator2d.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    p03_slam_core::p03_slam_core
)

add_library(${PROJECT_NAME}_node STATIC
  src/evaluation_node.cpp
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(${PROJECT_NAME}_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

add_executable(slam_evaluation_node
  src/evaluation_node_main.cpp
)
target_compile_features(slam_evaluation_node PUBLIC cxx_std_17)
target_link_libraries(slam_evaluation_node
  ${PROJECT_NAME}_node
)
ament_target_dependencies(slam_evaluation_node
  rclcpp
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS slam_evaluation_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_trajectory_evaluator2d
    test/test_trajectory_evaluator2d.cpp
  )
  target_link_libraries(test_trajectory_evaluator2d
    ${PROJECT_NAME}_core
  )
endif()

ament_package()
