cmake_minimum_required(VERSION 3.8)
project(tianrover_localization)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
add_executable(tf_authority_guard src/tf_authority_guard.cpp)
ament_target_dependencies(tf_authority_guard rclcpp std_msgs tf2_msgs)
install(TARGETS tf_authority_guard DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/ekf_inputs
  scripts/fastlio_io
  scripts/fastlio_prepare
  scripts/lidar_cloud_gate
  scripts/lidar_velocity
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ekf_inputs test/test_ekf_inputs.py)
  ament_add_pytest_test(test_lidar_frontend test/test_lidar_frontend.py)
endif()
ament_package()
