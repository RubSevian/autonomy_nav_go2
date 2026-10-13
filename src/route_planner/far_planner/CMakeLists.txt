cmake_minimum_required(VERSION 3.5)
project(far_planner)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visibility_graph_msg REQUIRED)

# Directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Build
set(SOURCES ${SOURCES}
  src/dynamic_graph.cpp
  src/planner_visualizer.cpp
  src/utility.cpp
  src/graph_planner.cpp
  src/contour_detector.cpp
  src/contour_graph.cpp
  src/map_handler.cpp
  src/scan_handler.cpp
  src/graph_msger.cpp
  src/terrain_planner.cpp
)

add_library(far_planner_runtime STATIC src/far_planner.cpp ${SOURCES})

ament_target_dependencies(far_planner_runtime rclcpp std_msgs std_srvs sensor_msgs nav_msgs diagnostic_msgs visualization_msgs pcl_conversions visibility_graph_msg tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs)

target_link_libraries(far_planner_runtime ${PCL_LIBRARIES} ${OpenCV_LIBS})
add_executable(${PROJECT_NAME} src/far_planner_node.cpp)
target_link_libraries(${PROJECT_NAME} far_planner_runtime)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(far_start_before_traversability_test tests/start_before_traversability_test.cpp)
  target_link_libraries(far_start_before_traversability_test far_planner_runtime)
  target_compile_options(far_start_before_traversability_test PRIVATE -UNDEBUG)
  add_test(NAME far_start_before_traversability_test COMMAND far_start_before_traversability_test)
  add_executable(far_vehicle_height_estimator_test tests/vehicle_height_estimator_test.cpp)
  target_include_directories(far_vehicle_height_estimator_test PRIVATE include)
  target_compile_options(far_vehicle_height_estimator_test PRIVATE -UNDEBUG)
  add_test(NAME far_vehicle_height_estimator_test COMMAND far_vehicle_height_estimator_test)
  add_executable(far_path_validation_test tests/path_validation_test.cpp)
  target_include_directories(far_path_validation_test PRIVATE include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(far_path_validation_test ${PCL_LIBRARIES})
  target_compile_options(far_path_validation_test PRIVATE -UNDEBUG)
  add_test(NAME far_path_validation_test COMMAND far_path_validation_test)
  add_executable(far_queued_goal_lifecycle_test tests/queued_goal_lifecycle_test.cpp)
  target_include_directories(far_queued_goal_lifecycle_test PRIVATE include)
  target_compile_options(far_queued_goal_lifecycle_test PRIVATE -UNDEBUG)
  add_test(NAME far_queued_goal_lifecycle_test COMMAND far_queued_goal_lifecycle_test)
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

ament_package()
