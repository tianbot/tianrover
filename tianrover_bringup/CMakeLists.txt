cmake_minimum_required(VERSION 3.8)
project(tianrover_bringup)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
ament_environment_hooks(env-hooks/99.tianrover.sh)
ament_package()
