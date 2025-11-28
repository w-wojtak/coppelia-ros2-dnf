#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sim
import time

class CoppeliaBridge(Node):
    def __init__(self):
        super().__init__('coppelia_bridge')
        self.get_logger().info("Starting CoppeliaSim bridge...")

        # Connect to CoppeliaSim
        self.clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        if self.clientID == -1:
            self.get_logger().error("Failed to connect to CoppeliaSim.")
            return

        self.get_logger().info(f"Connected to CoppeliaSim, clientID: {self.clientID}")

        # Load scene
        scene_path = '/home/rosuser/coppelia/scenes/project_test.ttt'
        sim.simxLoadScene(self.clientID, scene_path, 0, sim.simx_opmode_blocking)
        self.get_logger().info(f"Scene loaded: {scene_path}")

        # Cubes and ROS2 publishers
        self.cubes = ['Cuboid1', 'Cuboid2', 'Cuboid3']
        self.handles = []
        self.cube_publishers  = []

        for i, name in enumerate(self.cubes):
            # Get handle
            res, h = sim.simxGetObjectHandle(self.clientID, name, sim.simx_opmode_blocking)
            if res == sim.simx_return_ok:
                self.handles.append(h)
                self.get_logger().info(f"{name} handle: {h}")
            else:
                self.get_logger().error(f"Failed to get handle for {name}")

            # Create ROS2 publisher
            pub = self.create_publisher(Float32MultiArray, f'/cube{i+1}_pos', 10)
            self.cube_publishers.append(pub)

        # Move cubes sequentially and publish positions
        for i, h in enumerate(self.handles):
            pos = [0.1*i, 0.2*i, 0.3]
            sim.simxSetObjectPosition(self.clientID, h, -1, pos, sim.simx_opmode_blocking)
            self.get_logger().info(f"Moved {self.cubes[i]} to {pos}")

            # Read back position
            res, current_pos = sim.simxGetObjectPosition(self.clientID, h, -1, sim.simx_opmode_blocking)
            if res == sim.simx_return_ok:
                msg = Float32MultiArray()
                msg.data = current_pos
                self.cube_publishers[i].publish(msg)
                self.get_logger().info(f"Published {self.cubes[i]} position: {current_pos}")
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaBridge()
    rclpy.spin(node)
    if node.clientID != -1:
        sim.simxFinish(node.clientID)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
