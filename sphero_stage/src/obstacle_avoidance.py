import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from scipy.ndimage import rotate
from geometry_msgs.msg import Twist
from helper_function.utils import Vector2, get_agent_position, get_agent_velocity
from boid import *

class ObstacleAvoidance:
    def __init__(self, safe_distance):
        self.safe_distance = safe_distance
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

    def map_callback(self, msg):
        # Process the received map data
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_data = rotate(self.map_data, 90)  
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def check_for_obstacles(self, robot_position,robot_velocity):
        if self.map_data is None:
            return Vector2()  # No map data available yet

        # Convert robot position to map coordinates
        map_x, map_y = self.robot_to_map_coordinates(robot_position)

        # Check for obstacles within safe_distance
        for x in range(max(0, map_x - self.safe_distance[0]), min(self.map_data.shape[0], map_x + self.safe_distance[0])):
            for y in range(max(0, map_y - self.safe_distance[1]), min(self.map_data.shape[1], map_y + self.safe_distance[1])):
                if self.map_data[x, y] > 30:  # Assuming obstacle cells have a value greater than 30
                    # Calculate vector pointing away from the obstacle
                    obstacle_vector = Vector2(map_x - x, map_y - y)
                    if obstacle_vector.norm() < self.safe_distance[1]:
                        return obstacle_vector.normalize()

        return Vector2()  # No obstacles within safe distance

    def robot_to_map_coordinates(self, robot_position):
        # Convert robot position to map coordinates
        if self.map_resolution is None:
            return 0, 0  # Default value if map resolution is not available yet

        map_x = int((robot_position.x - self.map_origin[0]) / self.map_resolution)
        map_y = int((robot_position.y - self.map_origin[1]) / self.map_resolution)
        return map_x, map_y
    

    import math

    def get_rectangle_on_map(self, robot_position, robot_velocity, length, width): 
        """Get pixel coordinates of the rectangle on the map."""
        # Convert start point from world to map coordinates
        start_mx, start_my = self.robot_to_map_coordinates(robot_position)

        # Calculate end point based on length
        angle = math.atan2(robot_velocity.y, robot_velocity.x)
        end_mx = int(start_mx + length * math.cos(angle))
        end_my = int(start_my + length * math.sin(angle))

        # Calculate the perpendicular direction for the width
        perp_angle = angle + math.pi / 2
        dx_perp = width  * math.cos(perp_angle)
        dy_perp = width  * math.sin(perp_angle)

        # Calculate the four corners of the rectangle
        corners = [
            (start_mx + dx_perp, start_my + dy_perp),
            (start_mx - dx_perp, start_my - dy_perp),
            (end_mx + dx_perp, end_my + dy_perp),
            (end_mx - dx_perp, end_my - dy_perp)
        ]

        # Find min and max coordinates for the rectangle
        min_x = min(c[0] for c in corners)
        max_x = max(c[0] for c in corners)
        min_y = min(c[1] for c in corners)
        max_y = max(c[1] for c in corners)

        # Iterate through the rectangle area
        rectangle_pixels = []
        for x in range(int(min_x), int(max_x) + 1):
            for y in range(int(min_y), int(max_y) + 1):
                rectangle_pixels.append((x, y))

        return rectangle_pixels






# Integration with Boid class in boid.py
class BoidWithObstacleAvoidance(Boid):
    def __init__(self, *args, **kwargs):
        super(BoidWithObstacleAvoidance, self).__init__(*args, **kwargs)
        self.obstacle_avoidance = ObstacleAvoidance(safe_distance=[5,20])  # Adjust safe_distance as needed (it is in terms of rows and columns)

    def get_cmd_vel(self, agent_msg, target):
        # Original Boid behavior
        cmd = super(BoidWithObstacleAvoidance, self).get_cmd_vel(agent_msg, target)

        # Add obstacle avoidance behavior based on map data
        robot_position = get_agent_position(agent_msg)
        avoidance_vector = self.obstacle_avoidance.check_for_obstacles(robot_position)
        cmd.linear.x += avoidance_vector.x
        cmd.linear.y += avoidance_vector.y

        # Ensure the velocity does not exceed max speed
        velocity = Vector2(cmd.linear.x, cmd.linear.y)
        if velocity.norm() > self.max_speed_mag:
            velocity.set_mag(self.max_speed_mag)
            cmd.linear.x = velocity.x
            cmd.linear.y = velocity.y

        return cmd
