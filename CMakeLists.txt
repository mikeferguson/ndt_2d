cmake_minimum_required(VERSION 3.8)
project(ndt_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Primary ROS-agnostic library
add_library(ndt_2d_lib SHARED
  src/ndt_model.cpp
)

# Mapping node
add_library(ndt_2d_mapper SHARED
  src/ndt_mapper.cpp
)
ament_target_dependencies(ndt_2d_mapper
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(ndt_2d_mapper
  PLUGIN "ndt_2d::Map"
  EXECUTABLE ndt_2d_map_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-whitespace/braces" "-whitespace/newline")

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(ndt_model_tests test/ndt_model_tests.cpp)
  target_link_libraries(ndt_model_tests ndt_2d_lib)
  ament_target_dependencies(ndt_model_tests ${dependencies})
endif()

install(
  TARGETS
    ndt_2d_lib
    ndt_2d_mapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
