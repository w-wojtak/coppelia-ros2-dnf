#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import time

class CubeState:
    """Helper class to track the state of a single cube"""
    def __init__(self, name, initial_pos):
        self.name = name
        self.start_pos = initial_pos      # Where the move started
        self.current_pos = initial_pos    # Where it is right now
        self.last_move_time = time.time() # When it last changed position
        self.is_moving = False            # Is it currently in motion?
        self.has_reported = False         # Did we already report this specific move?

class CubeTracker(Node):
    def __init__(self):
        super().__init__('cube_tracker')
        
        self.get_logger().info("Tracker Started. Waiting for moves to finish...")
        
        # Dictionary to hold state for each cube
        self.cubes = {} 

        # Subscribe
        self.create_subscription(Float32MultiArray, '/cuboid1_pos', lambda m: self.update_cube(m, 'Cuboid1'), 10)
        self.create_subscription(Float32MultiArray, '/cuboid2_pos', lambda m: self.update_cube(m, 'Cuboid2'), 10)
        self.create_subscription(Float32MultiArray, '/cuboid3_pos', lambda m: self.update_cube(m, 'Cuboid3'), 10)

        # Check for settled cubes every 0.1 seconds
        self.timer = self.create_timer(0.1, self.check_settled)

    def get_dist(self, pos1, pos2):
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)

    def update_cube(self, msg, name):
        pos = msg.data # [x, y, z]
        
        # Initialize if new
        if name not in self.cubes:
            self.cubes[name] = CubeState(name, pos)
            return

        cube = self.cubes[name]
        
        # Check if position changed slightly (Jitter filter)
        if self.get_dist(pos, cube.current_pos) > 0.001:
            cube.current_pos = pos
            cube.last_move_time = time.time()
            cube.is_moving = True

    def check_settled(self):
        now = time.time()
        
        for name, cube in self.cubes.items():
            # If we haven't seen movement for 1.5 seconds, assume it stopped
            time_since_move = now - cube.last_move_time
            
            if cube.is_moving and time_since_move > 1.5:
                # It has settled!
                
                # Calculate total distance from Start to End
                total_dist = self.get_dist(cube.current_pos, cube.start_pos)
                
                # Only report if it moved a significant amount (> 10cm)
                if total_dist > 0.10:
                    self.get_logger().info(f"✅ FINAL: {name} finished moving. Distance: {total_dist:.2f}m. New Pos: {cube.current_pos}")
                    
                    # Reset for the next move
                    cube.start_pos = cube.current_pos
                
                cube.is_moving = False

def main(args=None):
    rclpy.init(args=args)
    node = CubeTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()