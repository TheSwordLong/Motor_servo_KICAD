cmake_minimum_required(VERSION 3.10...3.22)
project(robot_motor_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the pure-Python driver package. ament_cmake_python gives the same
# install layout ament_python would, without requiring the ament_python package
# (which this source-built Humble does not have).
ament_python_install_package(${PROJECT_NAME})

# Console entry point. Installed into lib/<package> like ament_python would do for
# a console_script, so `ros2 run robot_motor_driver robot_motor_node` works.
#
# `colcon build --symlink-install` links straight to the SOURCE file, so the source
# itself must carry the executable bit. A tree unpacked from a Windows tarball does
# not, and launch_ros filters the libexec directory with os.access(X_OK), which turns
# a non-executable entry point into "executable 'robot_motor_node' not found" at
# launch time even though the file is present.
#
# This has to run on EVERY build, not only at configure time: re-syncing the sources
# replaces the file and CMake does not necessarily re-configure, so an
# execute_process() alone silently stops protecting it.
if(UNIX)
  execute_process(COMMAND chmod +x ${CMAKE_CURRENT_SOURCE_DIR}/scripts/robot_motor_node)
  add_custom_target(robot_motor_node_executable ALL
    COMMAND chmod +x ${CMAKE_CURRENT_SOURCE_DIR}/scripts/robot_motor_node
    COMMENT "Ensuring scripts/robot_motor_node is executable for symlink-install")
endif()

install(PROGRAMS
  scripts/robot_motor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # These tests need no ROS runtime: protocol, framing, session, scheduler and the
  # simulated board are exercised offline. conftest.py puts the source tree on
  # sys.path so they also run from a plain checkout with `pytest test/`.
  ament_add_pytest_test(protocol_tests test/test_protocol.py TIMEOUT 180)
  ament_add_pytest_test(transport_tests test/test_transport.py TIMEOUT 180)
  ament_add_pytest_test(session_tests test/test_board_session.py TIMEOUT 240)
  ament_add_pytest_test(fake_board_tests test/test_fake_board.py TIMEOUT 240)
  ament_add_pytest_test(scheduler_tests test/test_scheduler.py TIMEOUT 300)
  ament_add_pytest_test(config_tests test/test_config.py TIMEOUT 120)
  ament_add_pytest_test(can_codec_tests test/test_transport_can.py TIMEOUT 120)

  # Node level tests need rclpy and the generated interfaces; they self-skip when
  # either is unavailable so the offline suite still passes elsewhere.
  ament_add_pytest_test(node_tests test/test_node.py TIMEOUT 600)
endif()

ament_package()
