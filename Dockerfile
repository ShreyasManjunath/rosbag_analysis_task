FROM ros:noetic-ros-base
RUN apt-get update && apt-get install python3-catkin-tools -y \
&& rm -rf /var/lib/apt/lists/*
SHELL ["/bin/bash", "-c"] 
ENV ROSBAG=

RUN source /opt/ros/noetic/setup.bash \
&& mkdir -p /ros_ws/src/rosbag_analysis_task/bag \
&& cd /ros_ws && catkin init

WORKDIR /ros_ws

COPY . /ros_ws/src/rosbag_analysis_task

RUN rosdep update && \
    rosdep install -y \
      --from-paths \
        /ros_ws/src/rosbag_analysis_task \
      --ignore-src && \
    rm -rf /var/lib/apt/lists/*

RUN catkin config \
--extend /opt/ros/$ROS_DISTRO && \
catkin build rosbag_analysis_task
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source /ros_ws/devel/setup.bash"

#COPY startpoint.sh /startpoint.sh
#RUN sudo chmod +x /startpoint.sh
#ENTRYPOINT ["/startpoint.sh"]




