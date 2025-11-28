#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sim # Make sure sim.py is in your path
import time

class CoppeliaBridge(Node):
    def __init__(self):
        super().__init__('coppelia_bridge')
        self.get_logger().info("Starting CoppeliaSim bridge...")

        # 1. Connect to CoppeliaSim
        # We use a loop to try connecting until it works
        self.clientID = -1
        while self.clientID == -1:
            self.clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
            if self.clientID == -1:
                self.get_logger().warn("Waiting for CoppeliaSim... (Make sure simulation is running)")
                time.sleep(1.0)

        self.get_logger().info(f"Connected! ClientID: {self.clientID}")

        # 2. Get Handles for Cubes
        # MAKE SURE these names match exactly what is in your Scene Hierarchy
        self.cube_names = ['Cuboid1', 'Cuboid2', 'Cuboid3']
        self.handles = {}
        self.publishers_dict = {}

        for name in self.cube_names:
            res, h = sim.simxGetObjectHandle(self.clientID, name, sim.simx_opmode_blocking)
            if res == sim.simx_return_ok:
                self.handles[name] = h
                
                # Initialize streaming mode for this object (required for fast reading)
                sim.simxGetObjectPosition(self.clientID, h, -1, sim.simx_opmode_streaming)
                
                # Create Publisher
                topic_name = f'/{name.lower()}_pos' # e.g., /cuboid1_pos
                self.publishers_dict[name] = self.create_publisher(Float32MultiArray, topic_name, 10)
                self.get_logger().info(f"Found {name}, publishing to {topic_name}")
            else:
                self.get_logger().error(f"Could not find object: {name}")

        # 3. Create a Timer to read positions (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Check connection
        if sim.simxGetConnectionId(self.clientID) == -1:
            self.get_logger().error("Lost connection to CoppeliaSim")
            return

        for name, handle in self.handles.items():
            # Read Position (Buffer mode is fast)
            res, pos = sim.simxGetObjectPosition(self.clientID, handle, -1, sim.simx_opmode_buffer)
            
            if res == sim.simx_return_ok:
                msg = Float32MultiArray()
                msg.data = pos # [x, y, z]
                self.publishers_dict[name].publish(msg)
                # Uncomment below to see spam in console
                # self.get_logger().info(f"{name}: {pos}")

def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    sim.simxFinish(node.clientID)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()