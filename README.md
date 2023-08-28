# Containerized ROS System

This repository contains a containerized ROS system consisting of an imaginary robot that moves in a 2D plane with random obstacles and an operator that sends commands to the robot to avoid the obstacles. The robot and the operator communicate with each other through ROS topics using encrypted String messages.\
The system is implemented in Python and containerized using Docker. The `Dockerfile` contains the instructions for building the Docker image, which is based on the official ROS Noetic Docker image. The `Makefile` contains the commands for building the Docker image, running the Docker container and managing the Docker image and container. The `interview_task` directory contains the ROS package for the system. 

## ROS Nodes

### Robot Node

The robot node is implemented in the `robot.py` file in the `interview_task/src` directory. The node publishes the robot's current heading and obstacle detection status to the `state` topic. The obstacle detection status is determined by a random number generator and the probability of detecting an obstacle is set by the `obstacle_prob` parameter. The node also listens for action commands on the `action` topic and performs the commanded action. The states and actions are published as encrypted String messages.

#### Subscribed Topics

- `action` (std_msgs/String)

#### Published Topics

- `state` (std_msgs/String)

#### Parameters

- `rate` (float): The rate at which the node publishes the robot's state in Hz. Default: 1.0
- `obstacle_prob` (float): The probability of the robot detecting an obstacle. Default: 0.25
- `secret_key` (string): The secret key used for ciphering. It must match the `secret_key` used for the operator node. Default: "secret_key"

### Operator Node

The operator node is implemented in the `operator.py` file in the `interview_task/src` directory. The node subscribes to the `state` topic and sends action commands to the robot node through the `action` topic. The actions are designed to make the robot continue in the current heading when there is no obstacle detected and to make the robot turn right or left with equal probability when there is an obstacle detected. The states and actions are published as encrypted String messages.

#### Subscribed Topics

- `state` (std_msgs/String)

#### Published Topics

- `action` (std_msgs/String)

#### Parameters

- `rate` (float): The rate at which the node publishes the robot's state in Hz. Default: 1.0
- `secret_key` (string): The secret key used for ciphering. It must match the `secret_key` used for the robot node. Default: "secret_key"

## ROS Launch File

The `nodes.launch` file in the `interview_task/launch` directory is used to launch the robot and operator nodes.

### Arguments

- `secret_key` (string): The secret key to use for ciphering the messages.

## Prerequisites

- Docker: [Installation Guide](https://docs.docker.com/get-docker/)

## Building and Running the System

1. Clone this repository
```
git clone https://github.com/sulthansf/dockerized-robot-operator-task.git
```
2. Change directory to the repository directory
```
cd dockerized-robot-operator-task
```
3. Build the Docker image and run the Docker container
```
make build run
```

## Using the System

- When the container is running, the nodes will be started automatically by default and the terminal will be attached to the container. The robot node will start publishing its state and the operator node will start sending action commands to the robot node.
- To detach from the container, press `Ctrl + P` followed by `Ctrl + Q`.

## Useful Commands
The following commands can be used to manage the Docker image and container while the current working directory is the repository directory.
- Attach to a running container:
```
make attach
```
- Open a new terminal in the container:
```
make exec
```
- Stop the container:
```
make stop
```
- Start the stopped container:
```
make start
```
- Remove the container and image:
```
make clean
```
- Rebuild the image and run the container:
```
make rebuild
```

## Cleaning Up

To clean up the Docker image and container, run the following command while the current working directory is the repository directory.
```
make clean
```