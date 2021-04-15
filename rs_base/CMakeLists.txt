cmake_minimum_required(VERSION 2.8.3)
project(rs_base)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

catkin_python_setup()

catkin_install_python(
	PROGRAMS
	  nodes/quadruped_controller_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})