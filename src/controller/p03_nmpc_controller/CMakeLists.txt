cmake_minimum_required(VERSION 3.12)
project(p03_nmpc_controller VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(casadi REQUIRED)
find_package(visualization_msgs)

add_executable(pid_controller src/pid_controller.cpp)
target_include_directories(pid_controller PUBLIC include)
ament_target_dependencies(pid_controller rclcpp geometry_msgs nav_msgs tf2)

add_executable(mpc_controller src/mpc_controller.cpp)
target_include_directories(mpc_controller PUBLIC include)
ament_target_dependencies(mpc_controller rclcpp geometry_msgs nav_msgs tf2 tf2_ros visualization_msgs)
target_link_libraries(mpc_controller casadi)

install(TARGETS pid_controller mpc_controller
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
