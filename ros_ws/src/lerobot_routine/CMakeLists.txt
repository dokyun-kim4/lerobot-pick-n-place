cmake_minimum_required(VERSION 3.8)
project(lerobot_routine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find yaml-cpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

add_executable(run_routine src/run_routine.cpp)
target_include_directories(run_routine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(run_routine PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(teach_routine src/teach_routine.cpp)
target_include_directories(teach_routine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS}
)

ament_target_dependencies(run_routine
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  moveit_visual_tools
)

ament_target_dependencies(teach_routine
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(teach_routine ${YAML_CPP_LIBRARIES})

install(TARGETS run_routine teach_routine
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
