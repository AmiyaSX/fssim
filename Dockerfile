FROM osrf/ros:noetic-desktop-full

ENV ROS_DISTRO=noetic

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO} 

# Set upp workspace
RUN mkdir -p /ws/src   

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary tools and dependencies
RUN apt-get update && \
    apt-get install -y python3-catkin-tools git python3-rosdep && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /ws
