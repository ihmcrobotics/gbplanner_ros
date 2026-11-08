cmake_minimum_required(VERSION 3.8)
project(planner_common)

add_compile_options(-Wall -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable -Wno-format -Werror=return-type)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
set(PLANNER_COMMON_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  pcl_conversions
  kdtree
  planner_msgs
  planner_semantic_msgs
)
foreach(dependency ${PLANNER_COMMON_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/ros2_compat.cpp
  src/graph_manager.cpp
  src/graph.cpp
  src/params.cpp
  src/trajectory.cpp
  src/random_sampler.cpp
  src/geofence_manager.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen kdtree::kdtree)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${PLANNER_COMMON_DEPENDENCIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Boost ${PLANNER_COMMON_DEPENDENCIES})
ament_package()
