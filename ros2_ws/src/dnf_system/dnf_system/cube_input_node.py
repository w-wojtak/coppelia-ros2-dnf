#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import math

class CubeState:
    def __init__(self, name, initial_pos):
        self.name = name
        self.reference_pos = list(initial_pos)  # Fixed reference (only resets when cube returns)
        self.current_pos = list(initial_pos)
        self.has_published = False

class CubeInput(Node):
    def __init__(self):
        super().__init__('cube_input')

        # ============ CONFIGURABLE PARAMETERS ============
        self.movement_threshold = 0.05   # 5 cm - triggers publish
        self.reset_threshold = 0.03      # 3 cm - cube must return within this to reset
        # =================================================

        self.get_logger().info(
            f"CubeInput started (trigger: {self.movement_threshold*100:.1f} cm, "
            f"reset: {self.reset_threshold*100:.1f} cm)"
        )

        self.cubes = {}

        self.cube_to_pos = {
            'Cuboid1': -20.0,
            'Cuboid2': 0.0,
            'Cuboid3': 20.0
        }

        self.dnf_pub = self.create_publisher(Float32, '/dnf_inputs', 10)

        for i, cube_name in enumerate(self.cube_to_pos.keys(), start=1):
            topic = f'/cuboid{i}_pos'
            self.create_subscription(
                Float32MultiArray,
                topic,
                lambda msg, c=cube_name: self.update_cube(msg, c),
                10
            )

        self.timer = self.create_timer(0.1, self.check_movement)

    def get_dist(self, p1, p2):
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

    def update_cube(self, msg, name):
        if not msg.data or len(msg.data) != 3:
            return

        pos = list(msg.data)

        if name not in self.cubes:
            self.cubes[name] = CubeState(name, pos)
            self.get_logger().info(f"Initialized {name} at {[f'{p:.3f}' for p in pos]}")
            return

        # Only update current position, NOT reference
        self.cubes[name].current_pos = pos

    def check_movement(self):
        for cube in self.cubes.values():
            distance = self.get_dist(cube.current_pos, cube.reference_pos)

            # Cube moved beyond threshold → publish ONCE
            if distance > self.movement_threshold and not cube.has_published:
                dnf_pos = self.cube_to_pos[cube.name]
                
                msg = Float32()
                msg.data = dnf_pos
                self.dnf_pub.publish(msg)
                
                self.get_logger().info(
                    f"🟢 {cube.name} moved {distance*100:.1f} cm → Publishing to DNF at x={dnf_pos}"
                )
                
                # Mark as published - won't publish again until reset
                cube.has_published = True

            # Cube returned near reference → reset for next detection
            elif distance < self.reset_threshold and cube.has_published:
                cube.has_published = False
                cube.reference_pos = list(cube.current_pos)  # Update reference
                self.get_logger().info(f"🔄 {cube.name} returned → Ready for next movement")

def main(args=None):
    rclpy.init(args=args)
    node = CubeInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()