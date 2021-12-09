# Simulation tools for Boston Dynamics' Spot

The package of Spot' controller based on [SpotMicro project](https://github.com/OpenQuadruped/spot_mini_mini).


## Install
Install docker:
Install Docker-CE using these [instructions](https://docs.docker.com/engine/install/ubuntu/)

Install nvidia-docker 
```bash
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
sudo apt-get install nvidia-docker2
sudo systemctl restart docker
```

Now we should build Spot container:

### Spot:

Init workspace

```bash
mkdir -p spot_ws/src
cd spot_ws/src
git clone -b spot_control https://github.com/SoftServeSAG/spot_simulation.git
```

Build docker images

```bash
cd spot_simulation/docker/ros_spot
chmod a+x build.bash
sudo ./build.bash 
```
Run docker container

```bash
chmod a+x run.bash
sudo ./run.bash
```

Inside docker container

```bash
cd /root/ws
apt-get update
rosdep install --from-paths src --ignore-src -r -y
rosdep check --from-paths . --ignore-src --rosdistro melodic
catkin config --init --extend /opt/ros/melodic   
catkin build
```

## Start environment
All following commands should be executed in specified containers:

```bash
cd ~/ws/src/spot_simulation/scripts
chmod a+x start.sh
chmod a+x session.yml
./start.sh
```
Now tmux session will start with all required tabs. All required commands may be added to the history, in order to simplify their usage.

## Start Gazebo
Launch world SoftServe office:
```bash
roslaunch rs_gazebo HQ.launch
```
If you want to use any other world use the following command:
```bash
roslaunch rs_gazebo HQ.launch world_name:="<world_name>"
```
To spawn robot:
```bash
roslaunch rs_gazebo robot.launch 
```
Spawn robot with the specific name and position:
```bash
roslaunch rs_gazebo robot.launch robot_name:="<spot_name>"  init_pose:="-x 0.0 -y 0.0 -z 0.0"
```
## Control
In case of forward kinematics, commands that set the joints angels are sent to actuators directly 
To control the robot with user-defined name, run the following commands:
```bash
roslaunch rs_control talker.launch 
```
To control robot with the user-defined name:
```bash
roslaunch rs_control talker.launch robot_name:="<spot_name>"
```
In case of inverse kinematics, the angels between joints are calculated with respect to the position of the end-effector. To execute the gait we use algorithm frrom [SpotMicro project](https://github.com/OpenQuadruped/spot_mini_mini).
In order to control Spot using inverse kinematics use the following commands: 

Start a controller:
```bash
roslaunch rs_inverse inverse.launch
```
To control robot with the user-defined name:
```bash
roslaunch rs_inverse inverse.launch robot_name:="<spot_name>"
```
Launch GUI to send commands:
```bash
roslaunch rs_inverse gui_inverse.launch
```
Run GUI for the robot with the user-defined name:
```bash
roslaunch rs_inverse gui_inverse.launch robot_name:="<spot_name>"
```
If you want control Spot using Twist() abd Pose() messages, then run the quadruped controller:
```bash
roslaunch rs_base quadruped_controller.launch 
```
You can specify the topics with velocity and pose commands, and the  topic in which gait command will be published:
```bash
roslaunch rs_base quadruped_controller.launch twist_topic_name:="" pose_topic_name:="" gait_topic_name:="<spot_name>/inverse_gait_input"
```

The robot can be controlled by a teleop for legged robots [teleop_legged_robots](https://github.com/SoftServeSAG/teleop_legged_robots). To teleoperate Spot run the following command:
```bash
roslaunch teleop_legged_robots teleop.launch 
```
To teleoperate Spot with the user-defined name run the following command:
```bash
roslaunch teleop_legged_robots teleop.launch robot_name:="<spot_name>"
```
