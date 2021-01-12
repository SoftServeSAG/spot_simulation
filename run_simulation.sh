echo "###############################################################################"
echo " RUN SIMULATION "
echo "###############################################################################"

BUCKET_SOURCE=borospotsource
BUCKET_OUTPUT=borospotoutput
JOB_ROLE=arn:aws:iam::448733523991:role/tborospotrole
NAME_ROBOT_APPLICATION=Spot1
NAME_SIMULATION_APPLICATION=Spot1_sim
MAX_JOB_DURATION=3600
BASE_DIR=`pwd`
ROS_SIM_DIR=$BASE_DIR/simulation_ws
ROS_APP_DIR=$BASE_DIR/robot_ws

echo "###############################################################################"
echo " RUNNING COLCON BUILD AND BUNDLE"
echo " NOTE: This will take 10-20 minutes for the first build/bundle, 1-2 minutes for subsequent build/bundle operations. "
echo "###############################################################################"

echo "Build and bunddle the simulation application"
cd $ROS_SIM_DIR
rosws update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
colcon bundle

echo "Build and bunddle the robot application"
cd $ROS_APP_DIR
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
colcon bundle


echo "Copy the robot application source bundle to your Amazon S3 bucket"

aws s3 cp $ROS_APP_DIR/bundle/output.tar s3://$BUCKET_SOURCE/spot1.tar

ROBOT_APPLICATION=$(aws robomaker create-robot-application --name $NAME_ROBOT_APPLICATION --sources s3Bucket=$BUCKET_SOURCE,s3Key=spot1.tar,architecture=X86_64 --robot-software-suite name=ROS,version=Melodic)

ROBOT_ARN=$(aws robomaker list-robot-applications --filter name="name",values="$NAME_ROBOT_APPLICATION" --query "robotApplicationSummaries[0].arn")

echo "Copy the simulation application source bundle to your Amazon S3 bucket"

aws s3 cp $ROS_SIM_DIR/bundle/output.tar s3://$BUCKET_SOURCE/spot1_sim.tar

SIMULATION_APPLICATION=$(aws robomaker create-simulation-application --name Spot1_sim --sources s3Bucket=$BUCKET_SOURCE,s3Key=spot1_sim.tar,architecture=X86_64 --robot-software-suite name=ROS,version=Melodic --simulation-software-suite name=Gazebo,version=9 --rendering-engine name=OGRE,version=1.x)

SIMULATION_ARN=$(aws robomaker list-simulation-applications --filter name="name",values="$NAME_SIMULATION_APPLICATION" --query "simulationApplicationSummaries[0].arn")

echo "###############################################################################"
echo " Create simulation job "
echo "###############################################################################"


aws robomaker create-simulation-job --max-job-duration-in-seconds $MAX_JOB_DURATION --iam-role $JOB_ROLE --output-location s3Bucket=$BUCKET_OUTPUT,s3Prefix=job --robot-applications application=$ROBOT_ARN,launchConfig='{packageName=rs_control,launchFile=inverse.launch}' --simulation-applications application=$SIMULATION_ARN,launchConfig='{packageName=rs_gazebo,launchFile=HQ.launch}'