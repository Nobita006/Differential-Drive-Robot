cmake_minimum_required(VERSION 3.8)
project(lawn_mower_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(robot_localization REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Register pytest tests
  ament_add_pytest_test(test_math_utils test/test_math_utils.py)
  ament_add_pytest_test(test_coverage_planner test/test_coverage_planner.py)
  ament_add_pytest_test(test_waypoint_generation test/test_waypoint_generation.py)
  ament_add_pytest_test(test_vision_processor test/test_vision_processor.py)
  ament_add_pytest_test(test_gemini_planner test/test_gemini_planner.py)
  ament_add_pytest_test(test_urdf test/test_urdf.py)
  ament_add_pytest_test(test_launch test/test_launch.py)
  ament_add_pytest_test(test_configs test/test_configs.py)
endif()

ament_package()

install(
  DIRECTORY launch urdf worlds maps config rviz models
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    mow_lawn.py
    simple_move.py
    vision_processor.py
    gemini_planner.py
    gemini_mow_planner.py
    gemini_mow_executor.py
    coverage_planner.py
  DESTINATION lib/${PROJECT_NAME}
)