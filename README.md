# Simulation tools for Boston Dynamics' Spot

This repository presents the adaptation of [towr](https://github.com/ethz-adrl/towr) for Spot from Boston Dynamics


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
git clone https://github.com/SoftServeSAG/spot_simulation.git
```

Build docker images

```bash
cd spot_simulation/docker/ros_melodic
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

## Run towr 
In order to launch a towr for Spot run:
```bash
roslaunch towr_ros towr_ros_spot.launch 
```
The guideline on towr is given [here](http://wiki.ros.org/towr). 
