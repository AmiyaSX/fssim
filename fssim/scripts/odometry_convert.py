#!/usr/bin/env python
import rospy

# ROS Msgs
from nav_msgs.msg import Odometry 
from fssim_common.msg import State


# Use the config variables
INPUT_ODOM_TOPIC = "/fssim/base_pose_ground_truth" # /fssim/base_pose_ground_truth (fssim)
OUTPUT_ODOM_TOPIC = "/dlio/odom_node/odom" # /dlio/odom_node/odom (MARS)

def cb_odometry_conversion(state):
    msg = Odometry()
    # Odometry.header (stamp, frame_id)
    msg.header.frame_id ="fssim_map"
    # Odometry.pose.pose.positionpoint
    msg.pose.pose.position.x = state.x
    msg.pose.pose.position.y = state.y
    msg.pose.pose.position.z = 0
    # TODO Odometry.pose.pose.orientation (confirm the mapping to quanternion here)
    msg.pose.pose.orientation.x = 0
    msg.pose.pose.orientation.y = 0
    msg.pose.pose.orientation.z = state.yaw 
    msg.pose.pose.orientation.w = 0
    # Odometry.twist.twist.linear
    msg.twist.twist.linear.x = state.vx
    msg.twist.twist.linear.y = state.vy
    msg.twist.twist.linear.z = 0
    # Odometry.twist.twist.angular
    msg.twist.twist.angular.x = 0
    msg.twist.twist.angular.y = 0
    msg.twist.twist.angular.z = state.r
    pub_odometry.publish(msg)

# FssimOdometryConversion node converts input from State msg(fssim) outputs Odometry msg(MARS)
if __name__ == '__main__':
    global pub_odometry
    rospy.init_node('fssim_odometry_conversion_node')
    rospy.loginfo("Fssim_odometry conversion node started")
    rospy.Subscriber(INPUT_ODOM_TOPIC, State, cb_odometry_conversion)
    pub_odometry = rospy.Publisher(OUTPUT_ODOM_TOPIC,
                          Odometry, queue_size=1)
    rospy.spin()