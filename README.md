# Simulation tools for Boston Dynamics' Spot

This repository helps to prepare AWS Robomaker environment and teleoperate Spot their.

## Setup 
To setup AWS Robomaker environment perform the following steps. 

- Create Source and Output Amazon S3 Buckets according to [guideline](https://docs.aws.amazon.com/robomaker/latest/dg/application-create-simjob.html) 

- Create a Simulation Job Role according to [guideline](https://docs.aws.amazon.com/robomaker/latest/dg/application-create-simjob.html). The template is copied here:
```bash
{
    "Version": "2012-10-17",
    "Statement": [
        {
            "Action": "s3:ListBucket",
            "Resource": [
                "arn:aws:s3:::my-input-bucket"
            ],
            "Effect": "Allow"
        },
        {
            "Action": [
                "s3:Get*",
                "s3:List*"
            ],
            "Resource": [
                "arn:aws:s3:::my-input-bucket/*"
            ],
            "Effect": "Allow"
        },
        {
            "Action": "s3:Put*",
            "Resource": [
                "arn:aws:s3:::my-output-bucket/*"
            ],
            "Effect": "Allow"
        },
        {
            "Action": [
                "logs:CreateLogGroup",
                "logs:CreateLogStream",
                "logs:PutLogEvents",
                "logs:DescribeLogStreams"
            ],
            "Resource": [
                "arn:aws:logs:*:account#:log-group:/aws/robomaker/SimulationJobs*"
            ],
            "Effect": "Allow"
        }
    ]
}
```
- Create a Development Environment according to [guideline](https://docs.aws.amazon.com/robomaker/latest/dg/gs-build.html)

- In the terminal section of your Development Environment run:
```bash
cd ~/environment
git clone -b spot_robomaker https://github.com/SoftServeSAG/spot_simulation.git

```
- Install colcon for building and bundling robot and simulation applications according to [guideline](https://docs.aws.amazon.com/robomaker/latest/dg/application-build-bundle.html) 
- In files run_simulation.sh specify the names of Buckets, Job Role, Robot Application and Simulation Application.

## Run Simulation  
To run the simulation run the following command in Cloud9 terminal:
```bash
cd spot_simulation
chmod +x run_simulation.sh
./run_simulation.sh
```

## Spot teleoperation
In AWS Robomaker environment select Simulation Job and then run Terminal from the Robot Application section. In this terminal execute the following command:

```bash
roslaunch rs_teleop teleop_spot.launch
```
