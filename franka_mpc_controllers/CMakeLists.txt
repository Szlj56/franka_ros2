cmake_minimum_required(VERSION 3.8)
project(franka_mpc_controllers)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/joint_impedance_sim_controller.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp controller_interface hardware_interface pluginlib trajectory_msgs)

pluginlib_export_plugin_description_file(
  controller_interface franka_mpc_controllers_plugin.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES franka_mpc_controllers_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()