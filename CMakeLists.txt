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
  src/ceres_solver.cpp
  src/constraint.cpp
  src/graph.cpp
  src/motion_model.cpp
  src/ndt_model.cpp
  src/occupancy_grid.cpp
  src/particle_filter.cpp
)
target_link_libraries(ndt_2d_lib ${CERES_LIBRARIES} Eigen3::Eigen "${cpp_typesupport_target}")
ament_target_dependencies(ndt_2d_lib ${dependencies})

# Mapping node
add_library(ndt_2d_mapper SHARED
  src/ndt_mapper.cpp
)
ament_target_dependencies(ndt_2d_mapper ${dependencies})
target_link_libraries(ndt_2d_mapper ndt_2d_lib "${cpp_typesupport_target}")
rclcpp_components_register_node(ndt_2d_mapper
  PLUGIN "ndt_2d::Mapper"
  EXECUTABLE ndt_2d_map_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(FILTERS "-whitespace/braces" "-whitespace/newline")

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(ceres_solver_tests test/ceres_solver_tests.cpp)
  target_link_libraries(ceres_solver_tests ndt_2d_lib)
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
  TARGETS
    ndt_2d_lib
    ndt_2d_mapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
