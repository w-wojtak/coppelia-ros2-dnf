#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class CubeTracker(Node):
    def __init__(self):
        super().__init__('cube_tracker')
        self.prev_positions = {'cube1': None, 'cube2': None, 'cube3': None}

        # Subscribe to each cube's position
        self.create_subscription(
            Float32MultiArray, '/cube1_pos', lambda msg: self.cube_callback(msg, 'cube1'), 10)
        self.create_subscription(
            Float32MultiArray, '/cube2_pos', lambda msg: self.cube_callback(msg, 'cube2'), 10)
        self.create_subscription(
            Float32MultiArray, '/cube3_pos', lambda msg: self.cube_callback(msg, 'cube3'), 10)

    def cube_callback(self, msg, cube_name):
        pos = msg.data
        prev = self.prev_positions[cube_name]

        if prev is None:
            self.get_logger().info(f"{cube_name} initial position: {pos}")
        elif any(abs(p - c) > 1e-5 for p, c in zip(prev, pos)):
            self.get_logger().info(f"{cube_name} moved from {prev} to {pos}")

        # Update the stored position
        self.prev_positions[cube_name] = pos

def main(args=None):
    rclpy.init(args=args)
    node = CubeTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
