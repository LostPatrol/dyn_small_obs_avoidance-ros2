# Build and export the original search library with ROS 2 dependencies.
cmake_minimum_required(VERSION 3.8)
project(path_searching)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree search io)

add_library(path_searching src/kinodynamic_astar.cpp)
target_include_directories(path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_options(path_searching PRIVATE -O3 -Wall)
ament_target_dependencies(path_searching rclcpp sensor_msgs pcl_conversions)
target_link_libraries(path_searching Eigen3::Eigen ${PCL_LIBRARIES})

install(TARGETS path_searching EXPORT export_path_searching
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_path_searching HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs pcl_conversions Eigen3 PCL)
ament_package()
