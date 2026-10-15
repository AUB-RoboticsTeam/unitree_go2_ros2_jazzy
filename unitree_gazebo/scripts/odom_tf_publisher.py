#!/usr/bin/env python3
"""Republish Gazebo's ground-truth body pose as /odom and the odom -> base TF.

This is the Gazebo Classic counterpart of the gz-sim OdometryPublisher used by
unitree_go2_ros2_jazzy, so RViz can use `odom` as its fixed frame.
"""
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdomTfPublisher(object):
    def __init__(self):
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.publish_tf = rospy.get_param("~publish_tf", True)

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.last_stamp = rospy.Time(0)
        rospy.Subscriber("ground_truth/odom", Odometry, self.callback, queue_size=10)

    def callback(self, msg):
        # TF rejects repeated timestamps for the same transform.
        if msg.header.stamp <= self.last_stamp:
            return
        self.last_stamp = msg.header.stamp

        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        self.odom_pub.publish(msg)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header = msg.header
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = msg.pose.pose.position.x
            tf_msg.transform.translation.y = msg.pose.pose.position.y
            tf_msg.transform.translation.z = msg.pose.pose.position.z
            tf_msg.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


if __name__ == "__main__":
    rospy.init_node("odom_tf_publisher")
    OdomTfPublisher()
    rospy.spin()
