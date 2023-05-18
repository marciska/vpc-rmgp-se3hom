# Getting Started

## Structure

The repository is organized in folders with files as follows:

```sh
├── docker/             # all docker related files to hust up a private network on your computer
├── matlab/             # the control logic resides here / MATLAB code
│   ├── data/           # GP model data
│   ├── lib/            # functions related to this project
│   ├── simulink/       # simulink projects
│   │   ├── generateData.slx  # is called when "generateGPdata.m" is run
│   │   ├── vfs_matlab.slx    # simulink model that runs a virtual pursuit simulation in MATLAB
│   │   └── vfs_unity.slx     # simulink model that runs a simulation within Unity over a ROS interface
│   ├── vpc_library/    # submodule that hosts the visual pursuit main building blocks
│   ├── generateGPdata.m    # function that generates GP data for our model
│   ├── init.m          # initializes all parameters
│   ├── kernel_computation_times.m     # script that generates the computation times figure
│   ├── main.m          # script that runs a simulation within MATLAB
│   └── startup.m       # loads all libraries and other files for simulation
├── ros/                # all ROS related files
└── unity-pursuit/      # Unity project files
```

> In order to run any file, please make sure that `startup.m` has been called to load all libraries (either automatically during the start of MATLAB, or manually).

## How to generate the Gaussian Process Data

Run the file `generateGPdata.m` to generate the Rigid Body Motion data for the GP models used in this repository.
It will also optimize the hyperparameters and create a few animation windows to help design a trajectory.

## Run the simulation in the paper

> Before every simulation, please ensure that the parameters have been initialized by calling the script `init.m`.

There are two ways to run the simulation.

### MATLAB simulation

This will run a simulation with switched trajectories within MATLAB only. Please note that factors like communication delays, air-resistance etc. are not considered here. To run this simulation, please start the Simulink file `vfs_matlab.slx`.

### Unity simulation

> **NOTE:** Our asset licences do not permit us to share the unity asset files like the bird, drones, and forest environment.
> However, all used assets are freely available and can be obtained by yourself.
> Please refer to the [assets guide](ASSETS.md) on how to obtain the assets.

This is a more practical simulation, as it will include factors that are likely to happen in the real world. Please ensure that you have already done the following:

- Created the ROS docker image as explained in [this guide](docker/README.md).
- Followed the unity setup guide [at the end of this file](#unity-setup-guide).
- Compiled the new ROS messages within the folder `ros/custom_msgs/` in MATLAB by the command:
    > `>> rosgenmsg('ros/custom_msgs/')`

Follow these steps in order to run the unity simulation:

1. (If no local ROS-environment is used) Setup the Docker-composed ROS network
    1. Start the ROS docker container as explained in [this guide](docker/README.md)

    2. Connect to the Docker-composed ROS network with [Tunnelblick](https://tunnelblick.net/downloads.html) or any other OpenVPN client.

    3. Open a new terminal and log into the ros docker container with:

        ```bash
        docker exec -it ros_melodic /bin/bash
        ```

        Then, start the server endpoint with the following command:

        ```bash
        roslaunch ros_tcp_endpoint endpoint.launch
        ```

2. Open the Unity Project and click on the triangle `Play` button. This will start the simulation, and we are ready to send the control inputs with MATLAB.

3. Open MATLAB, run `startup.m` and `init.m`, and connect to the ROS master within the Docker container:
    > `>> rosinit('docker-ros',11311,'NodeHost','192.168.255.6')`

4. Open the simulink file `vfs_unity.slx` and run the simulation.

## Unity Setup Guide

To setup Unity, please follow these steps:

1. Install Unity (`v2020.3+`).

2. Open the Unity Hub, select `Open -> Add project from disk` and navigate to the root folder of this repository to open the project folder `unityVPC`. Wait for Unity to open and install all dependencies.

3. Within Unity, open the `Package Manager` and ensure that `ROS-TCP-Connector` has been installed. If not, click the + button at the top left corner. Select "add package from git URL" and enter `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector` to install the [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) package. If you need a more in-depth guide, please follow the tutorial at the official [Unity Robotics Hub GitHub Project page](https://github.com/Unity-Technologies/Unity-Robotics-Hub).

4. Open `Robotics/ROS Settings` from the Unity menu bar, and set the `ROS IP Address` variable depending on your environment:

    - If you followed the guide for how to connect to a ROS docker container over a [virtual private network](docker/README.md#using-a-virtual-private-network), please change the `ROS IP Address` to `172.25.0.3`.
    - If you followed the guide for [how to remove the Docker isolation](docker/README.md#using-host-network-ubuntu-only), please leave the IP address as `127.0.0.1`.
    - If you have your own ROS environment, please change it to the PC that is running the ROS master.
