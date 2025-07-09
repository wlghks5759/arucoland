cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV 4.10.0 REQUIRED PATHS /usr/local NO_DEFAULT_PATH)

include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(/usr/local/lib)

add_executable(aruco_detector src/aruco_detector.cpp)

target_link_libraries(aruco_detector ${OpenCV_LIBS})

ament_target_dependencies(aruco_detector Eigen3 rclcpp px4_msgs sensor_msgs cv_bridge OpenCV)

target_compile_features(aruco_detector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS aruco_detector
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
