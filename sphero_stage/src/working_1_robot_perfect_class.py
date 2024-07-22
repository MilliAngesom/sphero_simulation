#!/usr/bin/env python3

import numpy as np
import pdb

import rospy
from nav_msgs.msg import Odometry
import time
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import math
from helper_function.utils import Vector2
from obs_awais import ObstacleAvoidance

from boid import Boid

class Robot_1_move:

    # take input the whole message from ros and return a vector of class Vector2
    def get_agent_velocity(self, agent):
        """Return agent velocity as Vector2 instance."""
        vel = Vector2()
        vel.x = agent.twist.twist.linear.x
        vel.y = agent.twist.twist.linear.y
        return vel

    # take input the whole message from ros and return a vector of class Vector2
    def get_agent_position(self, agent):
        """Return agent position as Vector2 instance."""
        pos = Vector2()
        pos.x = agent.pose.pose.position.x
        pos.y = agent.pose.pose.position.y
        return pos

    def update_target(self):       
        target = rospy.get_param('target')
        # self.pattern = rospy.get_param('pattern')
        if self.pattern == 'line_pattern':
            self.targets[0] = [target[0], target[1]]
            for i in range(1, self.num_of_robots):
                target = [target[0], target[1] + self.target_offset ] # + self.target_offset
                self.targets[i] = target

        if self.pattern == 'circular_pattern':
            center = [target[0], target[1]]
            angles = np.linspace(0, 2 * np.pi, self.num_of_robots, endpoint=False)

            for i in range(0, self.num_of_robots):
                x = center[0] + self.circle_radius * math.cos(angles[i])
                y = center[1] + self.circle_radius * math.sin(angles[i])
                self.targets[i] = [x, y]

    def odom_callback(self, msg):

        current_time = time.time()  # Get the current time
        if current_time - self.last_time_processed < 0.01:  # 0.1 seconds corresponds to 10 Hz
            return  # Not enough time has passed, so return without processing
        
        
        frame_id = msg.header.frame_id
        if frame_id not in self.cmd_vel_pub:
            self.cmd_vel_pub[frame_id] = rospy.Publisher("{}cmd_vel".format(frame_id[:9]), Twist, queue_size=10)
        id = frame_id[7] # string
        self.agent= self.agents[int(id)]
        self.agent_obs= self.agents_obs[int(id)]
        self.update_target()

        cmd = self.agent.get_cmd_vel(msg, self.targets[int(id)])

        p = self.get_agent_position(msg)
        vel = self.get_agent_velocity(msg) # np.deg2rad(self.get_agent_velocity(msg).arg())
        
        
        print("robot_position",p.x, p.y)
        
        obsAvodance_vel = self.agent_obs.main(np.array(self.targets[int(id)]), vel , np.array([p.x,p.y]) )
        
        gain = 2.0
        if obsAvodance_vel.x==0 and obsAvodance_vel.y==0:
            
            cmd.linear.x = cmd.linear.x + gain * obsAvodance_vel.x
            cmd.linear.y = cmd.linear.y + gain * obsAvodance_vel.y
        else:
            # print("I am here", obsAvodance_vel.x, obsAvodance_vel.y)
            cmd.linear.x = gain * obsAvodance_vel.x
            cmd.linear.y = gain * obsAvodance_vel.y

        
        # self.cmd_vel_pub[frame_id].publish(cmd)

        # plotting purpose only
        # self.steering_force = [cmd.linear.x, cmd.linear.y]
        # self.velocity = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]
        # steering_force_history.append(self.steering_force)
        # velocity_history.append(self.velocity)

        self.last_time_processed = current_time
        # self.rate.sleep()

    def __init__(self, max_speed, slowing_radius):

        self.last_time_processed = 0
        
        self.max_speed = max_speed
        self.slowing_radius = slowing_radius
        
        self.num_of_robots = rospy.get_param("/num_of_robots")-1
        self.target = rospy.get_param('target')
        self.target_offset = 0.8
        self.targets = [0] * self.num_of_robots
        self.velocity = [0.0, 0.0]  # Initial velocity
        self.pattern = rospy.get_param('pattern') #'circular_pattern' #'line_pattern'
        self.circle_radius = 2.0

        global velocity_history 
        global steering_force_history
        velocity_history = []
        steering_force_history = []
        self.frames = []
        self.cmd_vel_pub = {}
        # self.agent = None
        # self.agents = {}

        # self.agent_0 = Boid(initial_velocity_x = 0.0, initial_velocity_y = 0.0, 
        #                     max_speed_mag = self.max_speed, slowing_radius=self.slowing_radius)
        # rospy.Subscriber("robot_0/odom", Odometry, self.odom_callback)

        # self.rate = rospy.Rate(100)  # 10 Hz (you can adjust the rate as needed)


        # self.cmd_vel_pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        self.agents = [Boid(initial_velocity_x=0.0, initial_velocity_y=0.0, max_speed_mag=self.max_speed, 
                            slowing_radius=self.slowing_radius) for _ in range(self.num_of_robots)]
        self.agents_obs = [ObstacleAvoidance() for _ in range(self.num_of_robots)]

        [rospy.Subscriber("robot_{}/odom".format(i), Odometry, self.odom_callback) for i in range(self.num_of_robots)]
        # rospy.Subscriber("robot_0/odom", Odometry, self.odom_callback)


if __name__ == '__main__':
    try:
        rospy.init_node('rotate_robot_circularly')
        rospy.set_param('target', [1.0, -2.0])
        rospy.set_param('pattern', 'line_pattern')
        robot = Robot_1_move(max_speed=0.5, slowing_radius=1.0)
        rospy.spin()

        # Plotting
        # plt.figure(figsize=(10, 6))
        # plt.plot(steering_force_history, label='Steering Force')
        # plt.plot(velocity_history, label='Velocity')
        # plt.xlabel('Time Steps')
        # plt.ylabel('Value')
        # plt.legend()
        # plt.title('Steering Force and Velocity Over Time')
        # plt.show()

    except rospy.ROSInterruptException:
        pass