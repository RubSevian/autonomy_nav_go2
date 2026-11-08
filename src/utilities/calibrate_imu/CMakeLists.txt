cmake_minimum_required(VERSION 3.5)
project(calibrate_imu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include include/nlohmann)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(go2_sport_api REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(calibrate_imu src/calibrate_imu.cpp)
ament_target_dependencies(calibrate_imu rclcpp std_msgs sensor_msgs geometry_msgs unitree_api go2_sport_api)

install(TARGETS
  calibrate_imu
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}/
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Vendored nlohmann headers retain upstream formatting. Keep project sources
  # in the checks; only the two style tools exclude this third-party subtree.
  file(GLOB_RECURSE _vendor_json_headers "${CMAKE_CURRENT_SOURCE_DIR}/include/nlohmann/*.hpp")
  set(ament_cmake_uncrustify_ADDITIONAL_EXCLUDE ${_vendor_json_headers})
  set(ament_cmake_cpplint_ADDITIONAL_EXCLUDE ${_vendor_json_headers})
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

if(BUILD_TESTING AND TEST xmllint)
  # Resolve the ROS XSD locally, including when CTest has no network access.
  set_tests_properties(xmllint PROPERTIES
    ENVIRONMENT "XML_CATALOG_FILES=${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/lint/catalog.xml")
endif()
