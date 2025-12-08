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

add_executable(run_routine src/run_routine.cpp)
target_include_directories(run_routine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(run_routine PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(run_routine
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  moveit_visual_tools
)

install(TARGETS run_routine
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
