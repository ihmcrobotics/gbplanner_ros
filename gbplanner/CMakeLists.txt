cmake_minimum_required(VERSION 3.8)
project(gbplanner)

add_compile_options(-Wall -Wno-sign-compare -Wno-catch-value -Wno-switch -Wno-maybe-uninitialized -Wno-format -Werror=return-type)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)
set(GBPLANNER_DEPENDENCIES
  rclcpp
  ament_index_cpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf2
  pcl_conversions
  kdtree
  planner_common
  map_manager
  planner_msgs
  planner_semantic_msgs
  behaviortree_cpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
)
foreach(dependency ${GBPLANNER_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/gbplanner.cpp
  src/rrg.cpp
  src/gbplanner_rviz.cpp
  src/gbplanner_ros.cpp
  src/gbplanner_bt_nodes.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  map_manager::map_manager planner_common::planner_common ${OpenCV_LIBS} gflags glog)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${GBPLANNER_DEPENDENCIES})

add_executable(gbplanner_node src/gbplanner_ros_node.cpp)
target_link_libraries(gbplanner_node ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS gbplanner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${GBPLANNER_DEPENDENCIES})
ament_package()
