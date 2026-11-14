cmake_minimum_required(VERSION 3.8)
project(rover_gazebo)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_world test/test_world.py)
endif()
ament_package()
