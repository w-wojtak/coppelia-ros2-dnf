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

## Running Locally (without Docker)

1. Clone the repository:
```bash
git clone https://github.com/w-wojtak/coppelia-ros2-dnf.git
cd coppelia-ros2-dnf/ros2_ws
```

2. Build the workspace:
```bash
colcon build
source install/setup.bash
```

3. Launch the simulation:
```bash
ros2 launch dnf_system dnf_learn.launch.py
```

## ROS2 Topics

- `/cuboid1_pos`, `/cuboid2_pos`, `/cuboid3_pos`: Cube positions from CoppeliaSim

- `/dnf_inputs`: DNF input values from cube positions

## Notes

- Lua scripts for CoppeliaSim are committed in plain text (`.lua`) for reproducibility.

- Docker is recommended for easy setup, but the workspace can also be run directly on a ROS2-enabled machine.