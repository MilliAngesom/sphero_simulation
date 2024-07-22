#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Broadcast tf data during simulation.

tf data is produced from position of each robot received on Odometry messages.
It is used to visualize simulated robots in Rviz.
"""

import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

class SimulationTFBroadcaster:
    def __init__(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.num_of_robots = rospy.get_param("/num_of_robots")
        [rospy.Subscriber("robot_{}/odom".format(i), Odometry, self.callback) for i in range(self.num_of_robots)]
        

    def callback(self, msg):
        try:
            tf = geometry_msgs.msg.TransformStamped()
            tf.header.frame_id = "map"
            tf.child_frame_id = str(msg.header.frame_id)
            tf.header.stamp = msg.header.stamp

            tf.transform.translation.x = 0
            tf.transform.translation.y = 0
            tf.transform.translation.z = 0

            tf.transform.rotation.x = 0
            tf.transform.rotation.y = 0
            tf.transform.rotation.z = 0
            tf.transform.rotation.w = 1

            self.broadcaster.sendTransform(tf)
            print('modified')

        except BaseException as exc:
            print(exc)
            self.tfBuffer.clear()
            return
        


if __name__ == '__main__':
    rospy.init_node('simulation_tf')
    simulation_tf_broadcaster = SimulationTFBroadcaster()
    rospy.spin()




























    # def callback_2(self, msg):
    #     try:
    #         tf_2 = geometry_msgs.msg.TransformStamped()
    #         tf_2.header.frame_id = "map"
    #         tf_2.child_frame_id = msg.header.frame_id
    #         tf_2.header.stamp = msg.header.stamp

    #         tf_2.transform.translation.x = msg.pose.pose.position.x
    #         tf_2.transform.translation.y = msg.pose.pose.position.y
    #         tf_2.transform.translation.z = 0

    #         tf_2.transform.rotation.x = 0
    #         tf_2.transform.rotation.y = 0
    #         tf_2.transform.rotation.z = 0
    #         tf_2.transform.rotation.w = 1

    #         self.broadcaster.sendTransform(tf_2)
    #         print('modified')

    #     except BaseException as exc:
    #         print(exc)
    #         self.tfBuffer.clear()
    #         return