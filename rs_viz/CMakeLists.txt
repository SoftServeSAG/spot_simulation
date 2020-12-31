cmake_minimum_required(VERSION 2.8.3)
project(rs_viz)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY rviz launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
