cmake_minimum_required(VERSION 3.8)
project(rover_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rover_msgs REQUIRED)

add_library(rover_hardware SHARED src/rover_system.cpp)
target_include_directories(rover_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rover_hardware PUBLIC cxx_std_17)
target_link_libraries(rover_hardware PUBLIC
  hardware_interface::hardware_interface pluginlib::pluginlib rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle ${rover_msgs_TARGETS})

pluginlib_export_plugin_description_file(hardware_interface rover_hardware.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rover_hardware EXPORT export_rover_hardware
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rover_system test/test_rover_system.cpp)
  target_link_libraries(test_rover_system rover_hardware)
  target_link_libraries(test_rover_system hardware_interface::hardware_interface rclcpp::rclcpp ${rover_msgs_TARGETS})
endif()

ament_export_include_directories(include)
ament_export_libraries(rover_hardware)
ament_export_targets(export_rover_hardware)
ament_package()
