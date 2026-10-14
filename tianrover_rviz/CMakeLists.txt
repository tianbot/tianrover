cmake_minimum_required(VERSION 3.8)
project(tianrover_rviz)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
ament_package()
