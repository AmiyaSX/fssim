FROM osrf/ros:kinetic-desktop-full

WORKDIR /ws

# Update and install necessary tools and dependencies
RUN apt-get update && \
    apt-get install -y python-catkin-tools git python-rosdep && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

# Create src directory and clone the fssim repository
# RUN git clone https://github.com/AMZ-Driverless/fssim.git src/fssim
COPY ./ src/fssim

# Run update_dependencies script from the fssim directory
WORKDIR /ws/src/fssim

COPY ./update_dependencies.sh /
RUN chmod +x /update_dependencies.sh
RUN echo "y" | ./update_dependencies.sh

# Initialize catkin workspace and build
WORKDIR /ws

SHELL ["/bin/bash", "-c"]

RUN source /opt/ros/kinetic/setup.bash && \
    catkin init && \
    catkin build

# Append to bashrc for environment setup
RUN echo "source /ws/devel/setup.bash" >> ~/.bashrc

# Set default command to bash shell
CMD ["bash"]
