cmake_minimum_required(VERSION 3.10...3.22)
project(robot_motor_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(launch_tests test/test_launch.py TIMEOUT 300)
endif()

ament_package()
