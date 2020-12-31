echo "###############################################################################"
echo " CLOUD SUMMIT WORKSHOP: RUNNING COLCON BUILD AND BUNDLE (Open this shell script for command reference) "
echo " NOTE: This will take 10-20 minutes for the first build/bundle, 1-2 minutes for subsequent build/bundle operations. "
echo "###############################################################################"

cd $ROS_SIM_DIR
rosws update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
colcon bundle
