#!/usr/bin/env python3

import numpy as np
import pdb

import math
import rospy
from geometry_msgs.msg import Twist
from helper_function.utils import Vector2, angle_diff, get_agent_position, get_agent_velocity
import rospy
from geometry_msgs.msg import Twist
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
from tf.transformations import euler_from_quaternion

from helper_function.utils import Vector2
# import sphero_stage.src.sphero_stage.helper_function.utils as utils


class Boid(object):

    # use twist type for velocity and pose type message for position

    def __init__(self, initial_velocity_x, initial_velocity_y, max_speed_mag, slowing_radius): # , wait_count, start_count, frequency
        """Create an empty boid and update parameters."""
        self.position = Vector2()
        self.velocity = Vector2()
        self.mass = 0.18                # Mass of Sphero robot in kilograms (need to verify the exat weight)
        # self.wait_count = wait_count    # Waiting time before starting
        # self.start_count = start_count  # Time during which initial velocity is being sent
        # self.frequency = frequency      # Control loop frequency
        self.max_speed_mag = max_speed_mag #
        self.slowing_radius = slowing_radius #
        # Set initial velocity
        self.initial_velocity = Twist()
        self.initial_velocity.linear.x = initial_velocity_x
        self.initial_velocity.linear.y = initial_velocity_y

    
    def arrive(self, agent_msg, target):
        target_v = Vector2(target[0], target[1])
        desired_velocity_v = Vector2()
        self.position_v = get_agent_position(agent_msg) # agent position

        target_offset_v = target_v - self.position_v
        distance = target_offset_v.norm() 
        
        ramped_speed = (distance / self.slowing_radius)
        
        if distance < 1e-3:
            # print(f'position reached')
            return Vector2()
        else:
            desired_velocity_v.x = (ramped_speed / distance) * target_offset_v.x
            desired_velocity_v.y = (ramped_speed / distance) * target_offset_v.y
                    # ]
            if target_offset_v.norm() > self.max_speed_mag:
                desired_velocity_v.set_mag(self.max_speed_mag)
            # print(' desired_velocity_v', desired_velocity_v.x, desired_velocity_v.y)
            return desired_velocity_v
        
    def get_cmd_vel(self, agent_msg, target):

        self.steering_force_v = self.arrive(agent_msg, target)

        # steering_force_history.append(self.steering_force)
        # velocity_history.append(self.velocity)

        cmd = Twist()
        cmd.linear.x = self.steering_force_v.x
        cmd.linear.y = self.steering_force_v.y  # Adjust the angular velocity as needed
        return cmd

    # position and velocity vector components
    def set_pose(self, name, variable):
        rospy.set_param(name, variable)

    # position and velocity vector
    def get_pose(self, name):
        pose = rospy.get_param(name)
        new_vector = Vector2(pose[0], pose[1])
        return new_vector