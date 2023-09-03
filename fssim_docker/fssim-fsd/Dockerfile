FROM osrf/ros:kinetic-desktop-full

WORKDIR /root

# Install system dependencies and cleanup
RUN apt-get update && apt-get install -y \
    git \
    python-catkin-tools \
    python-rosdep \
    checkinstall && \
    rm -rf /var/lib/apt/lists/*

# Clone the fsd_skeleton repository
RUN git clone https://github.com/AMZ-Driverless/fsd_skeleton.git

WORKDIR /root/fsd_skeleton

# Use for subsequent RUN commands
SHELL [ "/bin/bash", "-c" ]

# Update dependencies and build workspace
# Note that we use the -f flag to install th    e fssim repository
# We also need to run the update_dependencies script for fssim hence the second script line
RUN source /opt/ros/kinetic/setup.bash && \
    yes | ./update_dependencies.sh -f && \
    yes | ./src/fssim/update_dependencies.sh && \
    catkin build && \
    source fsd_environment.sh

# Set default command to bash shell
CMD ["bash"]
