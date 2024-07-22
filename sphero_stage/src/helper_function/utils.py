#!/usr/bin/env python3

import numpy as np
import pdb
import math

import rospy
import logging
from geometry_msgs.msg import Twist

from helper_function import robot_boundary

class Vector2(object): # agent
    def __init__(self, x=0, y=0):
        """
        Initialize vector components.

        Args:
            x (float): x component of the vector
            y (float): y component of the vector
        """
        self.x = x
        self.y = y
    
    def __add__(self, other):
        if isinstance(other, self.__class__):
            return Vector2(self.x + other.x, self.y + other.y)
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x + other, self.y + other)

    def __sub__(self, other):
        if isinstance(other, self.__class__):
            return Vector2(self.x - other.x, self.y - other.y)
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x - other, self.y - other)

    def __div__(self, other):
        if isinstance(other, self.__class__):
            raise ValueError("Cannot divide two vectors!")
        elif isinstance(other, int) or isinstance(other, float):
            if other != 0:
                return Vector2(self.x / other, self.y / other)
            else:
                return Vector2()

    def __mul__(self, other):
        if isinstance(other, self.__class__):
            raise NotImplementedError("Multiplying vectors is not implemented!")
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x * other, self.y * other)

    def __rmul__(self, other):
        return self.__mul__(other)

    # def __str__(self):
    #     return "({: .5f}, {: 6.1f})".format(self.norm(), self.arg())
        # return "({: .3f}, {: .3f})".format(self.x, self.y)

    def __repr__(self):
        return "Vector2({0}, {1})\t norm = {2}\t arg = {3}".format(self.x, self.y, self.norm(), self.arg())

    
    # Robot velocity vector length in robot frame
    # Robot pose in world frame
    def norm(self):
        """Return the norm of the vector."""
        # print('x and y', self.x, self.y)
        return math.sqrt(pow(self.x, 2) + pow(self.y, 2))

    # Robot heading with velocity vector length in robot frameOR 
    # angle between robot position and origin with position vector in workd frame
    def arg(self):
        # print('x and y for angle ', self.x, self.y)
        """Return the angle of the vector."""
        return math.degrees(math.atan2(self.y, self.x))

    # Velocity vectors in robot frame
    def set_mag(self, value):
        """Set vector's magnitude without changing direction."""
        if self.norm() == 0:
            logging.warning('Trying to set magnitude for a null-vector! Angle will be set to 0!')
            self.x = 1
            self.y = 0
        else:
            self.normalize()
        self.x *= value
        self.y *= value

        # velocity vectors.
    def normalize(self, ret=False):
        """Normalize the vector."""
        d = self.norm()
        if d:
            if not ret:
                self.x /= d
                self.y /= d
            else:
                return Vector2(self.x / d, self.y / d)

    # angle setting in robot frame with velocity vector
    def set_angle(self, value):
        """Set vector's direction without changing magnitude."""
        if self.norm() == 0:
            logging.warning('Trying to set angle for a null-vector! Magnitude will be set to 1!')
            self.x = 1
            self.y = 0
        delta = angle_diff(self.arg(), value)
        self.rotate(delta)

    # veclocity vectors.
    def rotate(self, value):
        """Rotate vector by degrees specified in value."""
        value = math.radians(value)
        # print(value)
        self.x, self.y = math.cos(value) * self.x - math.sin(value) * self.y, \
                         math.sin(value) * self.x + math.cos(value) * self.y
        # print(self.x, self.y)



    # velocity vectors.

    def upper_limit(self, value):
        """Limit vector's maximum magnitude to given value."""
        if self.norm() > value:
            self.set_mag(value)

    # velocity vectors.
    def limit_lower(self, value):
        """Limit vector's minimum magnitude to given value."""
        if self.norm() < value:
            self.set_mag(value)

    def constrain(self, old_value, max_value): # old value is angle of the agent
        """Limit vector's change of direction to max_value from old_value."""
        desired_value = self.arg()
        delta = angle_diff(old_value, desired_value)
        if abs(delta) > max_value:
            value = angle_diff(desired_value, old_value + math.copysign(max_value, delta))
            self.rotate(value)


def angle_diff(from_angle, to_angle):
    diff = (to_angle - from_angle) % 360
    if diff >= 180:
        diff -= 360
    return diff


def pose_dist(pose1, pose2):
    """Return Euclidean distance between two ROS poses."""
    x1 = pose1.position.x
    y1 = pose1.position.y
    x2 = pose2.position.x
    y2 = pose2.position.y

    return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))

# position vectors
def pose_vector_dist(pose1, pose2):
    """Return Euclidean distance between two ROS poses."""
    x1 = pose1.x
    y1 = pose1.y
    x2 = pose2.x
    y2 = pose2.y

    return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))

# take input the whole message from ros and return a vector of class Vector2
def get_agent_velocity(agent):
    """Return agent velocity as Vector2 instance."""
    vel = Vector2()
    vel.x = agent.twist.twist.linear.x
    vel.y = agent.twist.twist.linear.y
    return vel

# take input the whole message from ros and return a vector of class Vector2
def get_agent_position(agent):
    """Return agent position as Vector2 instance."""
    pos = Vector2()
    pos.x = agent.pose.pose.position.x
    pos.y = agent.pose.pose.position.y
    return pos



# def dummy():

#     agent_1 = Vector2()
#     agent_1.x, agent_1.y= 1, 1
#     # print(agent_1.__repr__())
#     # print(agent_1.norm())
#     # print(agent_1.arg())
#     # agent_1.rotate(90)
#     # print(agent_1.norm())
#     # print(agent_1.arg())
#     # Set the agent_1 pose parameter
#     # agent_1.set_pose('agent_1_pose', [agent_1.x, agent_1.y])

#     # # Get the agent_1 pose parameter
#     agent_2 = Vector2()
#     agent_2.x, agent_2.y= 2, 2

#     tol = agent_2 * 2
#     print(tol.x, tol.y)
#     # print(agent_2.norm())
#     # print(agent_2.arg())

#     # print(pose_vector_dist(agent_1, agent_2))
#     # print(angle_diff(agent_1.arg(), agent_2.arg()))
#     # agent_2_1 = agent_2_1.get_pose('agent_1_pose') 
    
#     # print('agent_2_new_pose:', agent_2_1.x, agent_2_1.y)
   

# if __name__ == '__main__':
#     try:
#         rospy.init_node('rotate_robot_circularly')
#         robot = dummy()
#         # rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
