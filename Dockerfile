## Setup general ROS stuff
# https://registry.hub.docker.com/_/ros/

FROM ros:noetic-robot

ENV CATKIN_WS /root/catkin_ws
ENV PROJECT_NAME travesim_adapters

RUN mkdir -p ${CATKIN_WS}/src/
WORKDIR ${CATKIN_WS}

# Create catkin workspace
RUN /ros_entrypoint.sh catkin_make

COPY . src/${PROJECT_NAME}

RUN . devel/setup.sh && apt update --fix-missing && rosdep install ${PROJECT_NAME} -y

# Compile project
RUN /ros_entrypoint.sh catkin_make
