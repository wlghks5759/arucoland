cmake_minimum_required(VERSION 3.8)
project(precise_land)

set(CMAKE_CXX_STANDARD 20)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(px4_ros2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(precise_land src/precise_land.cpp)

ament_target_dependencies(precise_land rclcpp Eigen3 px4_ros2_cpp geometry_msgs tf2_ros tf2)

target_compile_features(precise_land PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS precise_land
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
