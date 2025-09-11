cmake_minimum_required(VERSION 3.8)
project(nav_pkg)

# === Warnings ===
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === ROS 2 dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# === Executable ===
add_executable(rtabmap_nav
  src/rtabmap_nav.cpp
)

# === Include directories ===
target_include_directories(rtabmap_nav PRIVATE
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# === ROS 2 target dependencies ===
ament_target_dependencies(rtabmap_nav
  rclcpp
  sensor_msgs
  geometry_msgs
  Eigen3
)

# === Install ===
install(TARGETS rtabmap_nav
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/config)

# === Lint / tests ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
