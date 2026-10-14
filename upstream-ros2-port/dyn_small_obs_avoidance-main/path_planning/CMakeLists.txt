# Build the planner and point transformer as ROS 2 executables.
cmake_minimum_required(VERSION 3.8)
project(path_planning)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(path_searching REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree search io)

add_executable(path_planning_node src/path_planning.cpp)
ament_target_dependencies(path_planning_node
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs trajectory_msgs
  visualization_msgs pcl_conversions path_searching
)
target_include_directories(path_planning_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(path_planning_node path_searching::path_searching Eigen3::Eigen ${PCL_LIBRARIES})

add_executable(goal_transform src/goalpoint_transformer.cpp)
ament_target_dependencies(goal_transform rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS path_planning_node goal_transform DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
