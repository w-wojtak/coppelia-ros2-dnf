# Coppelia-ROS2 DNF Simulation

This repository contains a ROS2-based simulation project integrating **CoppeliaSim** and a **Dynamic Neural Field (DNF)** system. It provides both a Docker setup for easy deployment and the ROS2 workspace for local development.

## Repository Structure

- **coppelia/**: Contains `.ttt` scene files and Lua scripts used in the simulation.  
- **project/docker/**: Dockerfile and `entrypoint.sh` for running the container.  
- **ros2_ws/src/**: ROS2 workspace source packages.  
- `.gitignore`: Excludes build artifacts, logs, and temporary files.

## Requirements

- Docker (for containerized use)  
- ROS2 Humble (if running outside Docker)  
- Python 3.10+  
- CoppeliaSim (for running scenes locally)

## Running with Docker

1. Build the Docker image:

```bash
cd project/docker
docker build -t coppelia-ros2-dnf .
```

2. Run the container:

```bash
docker run -it --rm --network host \
  -v /path/to/local/ros2_ws:/home/rosuser/ros2_ws \
  -v /path/to/local/coppelia:/home/rosuser/coppelia \
  coppelia-ros2-dnf
```

The `--network host` option ensures ROS2 nodes can communicate across containers and your host.

