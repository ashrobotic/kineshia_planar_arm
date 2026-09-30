cmake_minimum_required(VERSION 3.8)
project(planar_arm_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToTarget.srv"
)

# rosidl already installs a Python package of this name (the generated srv).
# Install the controller modules into that same package directory.
ament_get_python_install_dir(python_install_dir)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/planar_arm_control/
  DESTINATION ${python_install_dir}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(PROGRAMS
  planar_arm_control/controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME controller_node
)

install(PROGRAMS
  planar_arm_control/gui_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gui_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
