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

## Package and Node Descriptions

The project is organized as a modular ROS 2 system connecting a Dynamic Neural Field (DNF) model to CoppeliaSim. It includes two main packages: `dnf_system` and `coppelia_bridge`.

### dnf_system Package

This package implements the DNF architecture and handles input processing from the simulator.

- **cube_input_node**  
  Tracks cube positions from the simulator and publishes them as Gaussian-shaped inputs for the DNF learning node:
  - Subscribes to:
    - `/cuboid1_pos`
    - `/cuboid2_pos`
    - `/cuboid3_pos`
  - Publishes to:
    - `/dnf_inputs`

- **dnf_learning_node**  
  Implements the **learning phase** of the DNF architecture:
  - Subscribes to `/dnf_inputs` to receive processed inputs representing observed human actions.
  - Updates the **sequence memory field** (`u_sm`) and **task duration field** (`u_d`) in real time.
  - Produces internal state plots and stores activation histories for analysis.

- **dnf_recall_node**  
  Implements the **recall phase**:
  - Uses the learned memory and task duration fields to generate predicted action sequences.
  - Publishes these predictions as strings on `/dnf_predictions`.

### coppelia_bridge Package

This package interfaces with CoppeliaSim to handle both perception and robot control:

- **coppelia_bridge_node**  
  - Streams cube positions from CoppeliaSim to ROS topics during the learning phase.
  - Subscribes to `/dnf_predictions` during recall to trigger robot commands.
  - Sends commands to the simulated Franka arm via the Remote API.
  - Embedded Lua scripts in the scenes control the human model during learning and the robot arm during recall.

### System Flow

The system operates in two main phases:

1. **Learning Phase**
   - Cube positions are streamed from CoppeliaSim to ROS topics.
   - The `dnf_learning_node` updates the DNF memory and task duration fields.
2. **Recall Phase**
   - The `dnf_recall_node` publishes predicted actions to `/dnf_predictions`.
   - The bridge node translates these predictions into simulator commands, controlling the Franka arm.

This modular structure ensures a **clear separation of concerns**:
- Perception and input processing are handled independently from DNF computation.
- Robot control is encapsulated in the bridge node.

### ROS2 Topics Overview

| Topic                 | Publisher                 | Subscriber              | Description                                                                 |
|-----------------------|---------------------------|------------------------|-----------------------------------------------------------------------------|
| `/cuboid1_pos`        | coppelia_bridge_node      | cube_input_node        | Cube 1 position from simulator                                              |
| `/cuboid2_pos`        | coppelia_bridge_node      | cube_input_node        | Cube 2 position from simulator                                              |
| `/cuboid3_pos`        | coppelia_bridge_node      | cube_input_node        | Cube 3 position from simulator                                              |                                           |
| `/dnf_inputs`         | cube_input_node           | dnf_learning_node      | Gaussian-shaped DNF inputs derived from cube positions                      |
| `/dnf_predictions`    | dnf_recall_node           | coppelia_bridge_node   | Predicted action sequence strings for robot execution                       |



## Notes

- Lua scripts for CoppeliaSim are committed in plain text (`.lua`) for reproducibility.

- Docker is recommended for easy setup, but the workspace can also be run directly on a ROS2-enabled machine.