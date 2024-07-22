#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
from tf.transformations import euler_from_quaternion

class RobotPublisher:
    def __init__(self):
        rospy.init_node('robot_publisher')
        self.odom_publishers = {}
        self.marker_publishers = {}
        self.num_of_robots = rospy.get_param("/num_of_robots")
        [rospy.Subscriber("robot_{}/odom".format(i), Odometry, self.odom_callback) for i in range(self.num_of_robots)]

    def odom_callback(self, odom_msg):
        frame_id = odom_msg.header.frame_id

        # Publish Odometry message
        if frame_id not in self.odom_publishers:
            self.odom_publishers[frame_id] = rospy.Publisher("dummy_{}".format(frame_id), Odometry, queue_size=10)

        odom = Odometry()
        odom.header.stamp = odom_msg.header.stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = str(frame_id)

        odom.pose.pose.position.x = odom_msg.pose.pose.position.x
        odom.pose.pose.position.y = odom_msg.pose.pose.position.y
        odom.pose.pose.position.z = odom_msg.pose.pose.position.z

        odom.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
        odom.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
        odom.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
        odom.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

        odom.pose.covariance = [1.0, 0, 0.0, 0.0, 0.0, 0,
                                0, 1.0, 0.0, 0.0, 0.0, 0.0,  
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0, 0, 0.0, 0.0, 0.0, 0] 

        self.odom_publishers[frame_id].publish(odom)

        # Publish Marker message (Semicircle with lines)
        if frame_id not in self.marker_publishers:
            self.marker_publishers[frame_id] = rospy.Publisher("region_{}".format(frame_id), Marker, queue_size=10)

        marker = Marker()
        marker.header.frame_id = str(frame_id)[1:]
        marker.header.stamp = rospy.Time.now()
        marker.ns = "semicircle"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Line width
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Alpha (transparency)

        radius = 0.35  # Set your desired radius here
        num_points = 50  # Number of points to create the semicircle
        max_angle = 2.7  # Maximum angle in radians
        min_angle = -max_angle  # Minimum angle in radians

        robot_position = odom_msg.pose.pose.position
        robot_orientation = odom_msg.pose.pose.orientation
        robot_heading = self.calculate_orientation(robot_orientation)

        for i in range(num_points):
            angle = min_angle + (max_angle - min_angle) * i / num_points
            point = Point()
            point.x = robot_position.x + radius * math.cos(angle + robot_heading)
            point.y = robot_position.y + radius * math.sin(angle + robot_heading)
            point.z = robot_position.z
            marker.points.append(point)

        line2_start = Point()
        line2_start.x = robot_position.x
        line2_start.y = robot_position.y
        line2_start.z = 0.1
        line2_end = Point()
        line2_end.x = robot_position.x + radius * math.cos(min_angle + robot_heading)
        line2_end.y = robot_position.y + radius * math.sin(min_angle + robot_heading)
        line2_end.z = 0.1
        marker.points.append(line2_start)
        marker.points.append(line2_end)

        self.marker_publishers[frame_id].publish(marker)

    def calculate_orientation(self, orientation):
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return yaw

if __name__ == '__main__':
    try:
        RobotPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
