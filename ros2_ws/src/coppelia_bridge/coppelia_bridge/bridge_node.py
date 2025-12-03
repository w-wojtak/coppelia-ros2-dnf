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
        self.clientID = -1
        while self.clientID == -1:
            self.clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
            if self.clientID == -1:
                self.get_logger().warn("Waiting for CoppeliaSim... (make sure simulation is running)")
                time.sleep(1.0)

        self.get_logger().info(f"Connected! ClientID: {self.clientID}")

        # Cube handles and publishers
        self.cube_names = ['Cuboid1', 'Cuboid2', 'Cuboid3']
        self.handles = {}
        self.publishers_dict = {}  # rename to avoid conflict

        for name in self.cube_names:
            res, h = sim.simxGetObjectHandle(self.clientID, name, sim.simx_opmode_blocking)
            if res == sim.simx_return_ok:
                self.handles[name] = h
                # Start streaming for buffer reads
                sim.simxGetObjectPosition(self.clientID, h, -1, sim.simx_opmode_streaming)
                topic_name = f'/{name.lower()}_pos'
                self.publishers_dict[name] = self.create_publisher(Float32MultiArray, topic_name, 10)
                self.get_logger().info(f"Found {name}, publishing to {topic_name}")
            else:
                self.get_logger().error(f"Could not find object: {name}")

        # Give streaming some time to initialize
        time.sleep(0.2)

        # Timer to read positions at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if sim.simxGetConnectionId(self.clientID) == -1:
            self.get_logger().error("Lost connection to CoppeliaSim")
            return

        for name, handle in self.handles.items():
            res, pos = sim.simxGetObjectPosition(self.clientID, handle, -1, sim.simx_opmode_buffer)
            if res == sim.simx_return_ok:
                msg = Float32MultiArray()
                msg.data = pos  # [x, y, z]
                self.publishers_dict[name].publish(msg)
                # Uncomment to debug
                # self.get_logger().info(f"{name}: {pos}")

def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    sim.simxFinish(node.clientID)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
