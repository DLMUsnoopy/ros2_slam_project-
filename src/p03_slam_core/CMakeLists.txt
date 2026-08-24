cmake_minimum_required(VERSION 3.16)
project(p03_slam_core VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/grid_ray_tracing.cpp
  src/likelihood_field2d.cpp
  src/likelihood_field_matcher2d.cpp
  src/occupancy_grid2d.cpp
  src/occupancy_grid_updater.cpp
  src/odometry_buffer2d.cpp
  src/point2d.cpp
  src/pose2d.cpp
  src/slam_frontend2d.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)

  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(FILTERS "-legal/copyright")

  ament_add_gtest(test_package_smoke
    test/test_package_smoke.cpp
  )
  target_link_libraries(test_package_smoke ${PROJECT_NAME})

  ament_add_gtest(test_pose2d
    test/test_pose2d.cpp
  )
  target_link_libraries(test_pose2d ${PROJECT_NAME})

  ament_add_gtest(test_point2d
    test/test_point2d.cpp
  )
  target_link_libraries(test_point2d ${PROJECT_NAME})

  ament_add_gtest(test_occupancy_grid2d
    test/test_occupancy_grid2d.cpp
  )
  target_link_libraries(test_occupancy_grid2d ${PROJECT_NAME})

  ament_add_gtest(test_grid_ray_tracing
    test/test_grid_ray_tracing.cpp
  )
  target_link_libraries(test_grid_ray_tracing ${PROJECT_NAME})

  ament_add_gtest(test_occupancy_grid_updater
    test/test_occupancy_grid_updater.cpp
  )
  target_link_libraries(test_occupancy_grid_updater ${PROJECT_NAME})

  ament_add_gtest(test_odometry_buffer2d
    test/test_odometry_buffer2d.cpp
  )
  target_link_libraries(test_odometry_buffer2d ${PROJECT_NAME})

  ament_add_gtest(test_likelihood_field2d
    test/test_likelihood_field2d.cpp
  )
  target_link_libraries(test_likelihood_field2d ${PROJECT_NAME})

  ament_add_gtest(test_likelihood_field_matcher2d
    test/test_likelihood_field_matcher2d.cpp
  )
  target_link_libraries(test_likelihood_field_matcher2d ${PROJECT_NAME})

  ament_add_gtest(test_slam_frontend2d
    test/test_slam_frontend2d.cpp
  )
  target_link_libraries(test_slam_frontend2d ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
