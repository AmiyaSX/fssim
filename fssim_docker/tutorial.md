# FSSIM Tutorial

This document is a tutorial for running the FS Simulation using Docker.

## Table of Contents

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Setup](#setup)
  - [Building the Docker image](#building-the-docker-image)
  - [Running the Docker container](#running-the-docker-container)
  - [X11 Server](#x11-server)
    - [Linux](#linux)
    - [Windows](#windows)
    - [MacOS](#macos)
  - [Ensure the Simulation GUI is Working](#ensure-the-simulation-gui-is-working)
- [Simulating the Pipeline](#simulating-the-pipeline)
- [CI Automated Testing](#ci-automated-testing)

## Introduction

This directory contains two subdirectories:

- [fssim-base](fssim-base): contains a Dockerfile for building the simulation without any pipeline
- [fssim-fsd](fssim-fsd): contains a Dockerfile for building the simulation with the AMZ [fsd pipeline](https://github.com/AMZ-Driverless/fsd_skeleton/).

The `fssim-base` directory would be the starting point for building the simulation with our own pipeline. For testing purposes (for either Docker or ROS), you can use the `fssim-fsd` directory to build the simulation with the fsd pipeline.

The following sections will guide you through the process of building and running the simulations using Docker. The main focus of this document will be on the `fssim-base` Dockerfile since this is the image that you will be using to build and test our pipeline to the simulation as well as using it in the CI/CD framework to automate headless testing of our pipeline.

## Prerequisites

To run the simulation, you need to have the following installed:

- Docker
- X11 display server

## Setup

### Building the Docker image

To run the simulation, you need to build the Docker image first. To do so, run the following command (in the [fssim-base](fssim-base) directory):

```bash
docker build -t fssim-base .
```

This will build the Docker image and tag it as `fssim-base`. You can now create Docker containers using this image.

### Running the Docker container

Once the Docker image is built and the X11 display server is running, you can proceed to run the simulation container.

In UNIX environments, you can run the following command:

```bash
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --name fssim fssim-base
```

### X11 Server

To display the simulation GUI, you need to have a display server running on your machine so that the Docker container can access it and forward the GUI to your screen. The following subsections will guide you through the process of setting up the X11 display server on your machine for different operating systems (which should be configured outside of the Docker container - so open a new terminal or exit out of the Docker container).

*NOTE:* When you need to expose the X11 display server to a Docker container securely, you can limit the access by allowing only that specific container to connect to the host's X11 server. One way to achieve this is by using the hostname or container ID to make a more specific `xhost` entry. Exposing the X11 server to all local users != recommended as it is a security risk. Read more about GUI in ROS containers [here](https://wiki.ros.org/docker/Tutorials/GUI).

#### Linux

Follow these steps to set up the X11 display server on Linux the secure way:

1. **Get the Container ID**

    You can obtain the container's ID:

    ```bash
    containerId=$(docker ps -q --filter name=fssim)
    ```

    *Replace 'fssim' with the name of your container*

2. **Add the Hostname or Container ID to xhost**

    Now, you can add this container ID to `xhost` to allow only this specific container to connect to your X11 server:

    ```bash
    xhost +local:$containerId
    ```

3. **Run or Restart Your Docker Container**

    If your container is already running, you might need to restart it for changes to take effect. If not, run your container as you normally would, ensuring that it is configured to use the host's X11 server.

    ```bash
    docker restart $containerId
    ```

    or

    ```bash
    docker run -e DISPLAY=$DISPLAY ... # Your usual flags and image name here
    ```

4. **OPTIONAL: Remove the xhost Entry After Use**

    After you are done using the Docker container, it's a good practice to remove its access to the X11 server:

    ```bash
    xhost -local:$containerId
    ```

By following these steps, you'll limit X11 access to only the specified Docker container, enhancing your system's security.

*NOTE*: The unsecure way of exposing the X11 server to all local users is by running the following command:

```bash
xhost +local:
```

although this is **not recommended**.

Next, [Ensure the Simulation GUI is Working](#ensure-the-simulation-gui-is-working).

### Windows

1. **Install and Setup Xming for Docker Containers:**

    First, download and install [Xming](https://sourceforge.net/projects/xming/) on your Windows machine. Then follow [these](https://smc181002.medium.com/running-gui-application-in-docker-with-xming-e9d81c5600e) instructions to set up Xming.

2. **Run Docker Container:**

    Start your Docker container, setting the `DISPLAY` environment variable to point to the Xming server running on your Windows machine. Replace `fssim-base` with the image you're using and `192.168.1.2` with the IP of your Windows machine.

    ```bash
    docker run -e DISPLAY=192.168.1.2:0.0 fssim-base
    ```

    Or, if the container is already running:

    ```bash
    docker exec -e DISPLAY=192.168.1.2:0.0 -it container_id /bin/bash
    ```

    In these commands, `192.168.1.2` should be replaced with your own Windows IP address, and `container_id` with the actual Docker container ID.

By following these steps, you should be able to run GUI-based applications from inside your Docker container and have them display on your Windows machine via Xming.

Next, [Ensure the Simulation GUI is Working](#ensure-the-simulation-gui-is-working).

### MacOS

On macOS, you can use XQuartz as the X11 server.

1. **Install XQuartz**

    Download and install [XQuartz](https://www.xquartz.org/).

2. **Start XQuartz and Enable Connections**

    Run XQuartz and then enable the "Allow connections from network clients" option from XQuartz settings.

3. **Get the Container ID**

    Open Terminal and get the Docker container ID.

    ```bash
    containerId=$(docker ps -q --filter name=fssim)
    ```

4. **Add the Container ID to xhost**

    macOS, like Linux, has a native `xhost` command.

    ```bash
    xhost +local:$containerId
    ```

5. **Run or Restart Your Docker Container**

    If your container is already running, you might need to restart it for the changes to take effect.

    ```bash
    docker restart $containerId
    ```

6. **OPTIONAL: Remove the xhost Entry After Use**

    After you are done using the Docker container, it's a good practice to remove its access to the X11 server.

    ```bash
    xhost -local:$containerId
    ```

By following these steps according to your operating system, you should be able to securely display GUI applications from a Docker container.

### Ensure the Simulation GUI is Working

Try running the following command *in the Docker container* to see if the GUI is working:

```bash
roslaunch fssim auto_fssim.launch
```

The simulation GUI should pop up. It might take a while for the GUI and all elements to load. Once all elements have been loaded, you might need to uncheck and recheck the *FSSIM Map* and *RobotModel* checkboxes in the left panel to make the map and vehicle model appear (this is a heavy operation and might take some time to load the vehicle model).

## Simulating the Pipeline

Once the FSSIM base Docker container is running and the simulation GUI is working, you can proceed to run the pipeline. For this, you will need to build the container with a mounted volume to the pipeline code. To do so, run the following command (for UNIX environments - see the [Windows](#windows) section for Windows):

```bash
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v /path/to/pipeline:/ws/src/ --name fssim fssim-base
```

where `/path/to/pipeline` is the path to the pipeline code on your machine. This will mount the pipeline code to the `/ws/src` directory in the Docker container. Since the pipeline code is mounted to the Docker container's src directory in the ROS workspace, the mounted pipeline code (in `/path/to/pipeline`) should contain ROS packages.

*NOTE:* Remember that changes made to the pipeline code on your machine will be reflected in the Docker container and vice versa - so be careful when making changes or moving files in the pipeline code in the Docker container.

## CI Automated Testing

**TODO**
