cmake_minimum_required(VERSION 3.8)
project(map_manager)

add_compile_options(-Wall -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable -Wno-format -Werror=return-type)
add_compile_definitions(RAY_CAST_METHOD=0)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
set(MAP_MANAGER_DEPENDENCIES
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  pcl_conversions
  voxblox
  voxblox_ros
  planner_common
  planner_msgs
)
foreach(dependency ${MAP_MANAGER_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/map_manager.cpp
  src/voxblox/voxblox_common_impl.cpp
  src/voxblox/voxblox_alt_impl.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC voxblox_ros::voxblox_ros planner_common::planner_common)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${MAP_MANAGER_DEPENDENCIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${MAP_MANAGER_DEPENDENCIES})
ament_package()
