cmake_minimum_required(VERSION 3.8)
project(localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Find Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(fake_encoder
  src/fake_encoder.cpp
)

add_executable(fake_odom
  src/fake_odom.cpp
)

ament_target_dependencies(fake_encoder
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

ament_target_dependencies(fake_odom
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS
  fake_encoder
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
  fake_odom
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()