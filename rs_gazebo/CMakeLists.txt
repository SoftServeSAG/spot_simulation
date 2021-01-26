cmake_minimum_required(VERSION 2.8.3)
project(rs_gazebo)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS gazebo_ros gazebo_ros_control rs_description
)

install(FILES
  launch/HQ.launch
  worlds/ss_hq4.world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  
install(DIRECTORY launch meshes models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
