cmake_minimum_required(VERSION 2.8.3)
project(rs_control)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

catkin_install_python(
	PROGRAMS
	  scripts/talker.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})