cmake_minimum_required(VERSION 3.8)
project(ndt_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Constraint.msg"
  "msg/Scan.msg"
  "srv/Configure.srv"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(
  include
  ${CERES_INCLUDES}
)

set(dependencies
  angles
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_components
  rosbag2_cpp
  sensor_msgs
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

# Primary library
add_library(ndt_2d_lib SHARED
  src/constraint.cpp
  src/motion_model.cpp
  src/ndt_model.cpp
  src/occupancy_grid.cpp
  src/particle_filter.cpp
  src/scan.cpp
)
target_link_libraries(ndt_2d_lib Eigen3::Eigen)
ament_target_dependencies(ndt_2d_lib ${dependencies})

# Scan Matcher NDT Plugin
add_library(scan_matcher_ndt SHARED
  src/scan_matcher_ndt.cpp
)
target_link_libraries(scan_matcher_ndt ndt_2d_lib)
ament_target_dependencies(scan_matcher_ndt ${dependencies})

# Mapping node
add_library(ndt_2d_mapper SHARED
  src/ceres_solver.cpp
  src/graph.cpp
  src/ndt_mapper.cpp
)
target_link_libraries(ndt_2d_mapper
  ndt_2d_lib
  ${CERES_LIBRARIES}
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)
ament_target_dependencies(ndt_2d_mapper ${dependencies})
rclcpp_components_register_node(ndt_2d_mapper
  PLUGIN "ndt_2d::Mapper"
  EXECUTABLE ndt_2d_map_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-whitespace/braces" "-whitespace/newline")

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(ceres_solver_tests test/ceres_solver_tests.cpp)
  target_link_libraries(ceres_solver_tests ndt_2d_lib ndt_2d_mapper)
  ament_target_dependencies(ceres_solver_tests ${dependencies})

  ament_add_gtest(graph_tests test/graph_tests.cpp)
  target_link_libraries(graph_tests ndt_2d_lib ndt_2d_mapper)
  ament_target_dependencies(graph_tests ${dependencies})

  ament_add_gtest(ndt_model_tests test/ndt_model_tests.cpp)
  target_link_libraries(ndt_model_tests ndt_2d_lib)
  ament_target_dependencies(ndt_model_tests ${dependencies})

  ament_add_gtest(particle_tests test/particle_tests.cpp)
  target_link_libraries(particle_tests ndt_2d_lib ndt_2d_mapper)
  ament_target_dependencies(particle_tests ${dependencies})
endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS
    ndt_2d_lib
  EXPORT export_ndt_2d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    ndt_2d_mapper
    scan_matcher_ndt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(ndt_2d plugins.xml)
ament_export_include_directories(include)
ament_export_libraries(ndt_2d_lib)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
