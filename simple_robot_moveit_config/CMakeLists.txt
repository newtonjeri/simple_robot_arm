cmake_minimum_required(VERSION 3.22)
project(simple_robot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)


add_executable(move_arm_node src/move_arm.cpp)

ament_target_dependencies(move_arm_node moveit_ros_planning_interface moveit_msgs rclcpp)
target_include_directories(move_arm_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(move_arm_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS move_arm_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
