cmake_minimum_required(VERSION 3.8)
project(tianrover_calibration)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/calibrate
  scripts/calibration_bag
  scripts/calibration_motion
  scripts/calibration_parameters
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_calibration test/test_calibration.py)
  ament_add_pytest_test(test_calibration_alignment test/test_calibration_alignment.py)
  ament_add_pytest_test(test_calibration_application test/test_calibration_application.py)
  ament_add_pytest_test(test_calibration_bag test/test_calibration_bag.py)
  ament_add_pytest_test(test_calibration_cli test/test_calibration_cli.py)
  ament_add_pytest_test(test_calibration_motion test/test_calibration_motion.py)
endif()
ament_package()
