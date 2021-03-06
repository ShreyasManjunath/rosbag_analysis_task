# Rosbag analysis task

## Task : To automate the error detection and analysis with rosbag as input.
<i>Shreyas Manjunath</i>
***

Topics used in this task:
- /long_term_slam/error_code
- /nav_controller/error_code
- /stop_signal

## How to build and run the rosbag_analysis_task node?
- Create a ROS workspace.
- Add this package "rosbag_analysis_task"to the ros_ws/src directory
- catkin build rosbag_analysis_task
- Run the following command to launch the rosbag_analysis_task node and the chosen rosbag.

```
roslaunch rosbag_analysis_task analyse.launch rosbag_file_name:=$ABSOLUTE_PATH$
```
For example:

```
roslaunch rosbag_analysis_task analyse.launch rosbag_file_name:=/home/shreyas/04_Rosbag_Analysis_Task/dummy_env-agv-50231.agv-2020-10-01T082312+0200_2020-10-01-11-28-17_37.bag
```

## Output

A log file named "report.log" is generated by the node with the errors and corresponding timestamp under the /data directory inside the package.

## Docker Support

As part of the task 2, the ability to run on different machine or on the customer's system can be achieved by docker containers.

How to build and run the node inside a docker container?
- Navigate to the package directory, for example: ~/ros_ws/src/rosbag_analysis_task
- This folder contains Dockerfile, docker-compose.yml and .env files that are required for building and running the package inside the container.
- To build the image, run the following command:
```
sudo docker build -t <your_choice_of_docker_image_name> .
```
For example:
```
sudo docker build -t rosbag_analysis .
``` 
- Open .env file located in the same directory. It has 3 fields namely:
```
ROSBAG_NAME=xyz.bag
BAG_LOCATION=/home/user_name/folder_name
OUTPUT_LOCATION=/home/user_name/folder_name
```
ROSBAG_NAME -> Rosbag name.\
BAG_LOCATION -> Rosbag location on the user's side (Used as docker volume).\
OUTPUT_LOCATION -> Desired output location on the user's side to obtain the "report.log" file.

- Further, docker-compose command is used to bringup the container for launching the container and running the node.
```
sudo docker-compose up -d
```

Look for "report.log" file at the previously specified location.