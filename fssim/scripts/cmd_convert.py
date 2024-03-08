#!/usr/bin/env python
import rospy

# ROS Msgs
from fssim_common.msg import dv_control_target
from fssim_common.msg import Cmd

# Use the config variables
INPUT_CMD_TOPIC = rospy.get_param('conversion/topics/origin_cmd_topic') # /ros2can/send/dv_control_target (MARS)
OUTPUT_CMD_TOPIC = rospy.get_param('conversion/topics/target_cmd_topic') # /fssim/cmd (fssim)

def cb_dv_control_target_conversion(dv_control_target):
    msg = dv_control_target()
    crt_speed = dv_control_target.dv_speed_target
    # TODO mapping the range to -1~1
    msg.dv = crt_speed
    msg.delta = dv_control_target.dv_steering_angle_target / 90 
    pub_dv_control.publish(msg)

# FssimCmdConversion node converts input from dv_control_target msg(MARS) outputs Cmd msg(fssim)
if __name__ == '__main__':
    global pub_dv_control
    global pub_odometry
    rospy.init_node('fssim_cmd_conversion_node')
    rospy.loginfo("Fssim_cmd_conversion node started")
    rospy.Subscriber(INPUT_CMD_TOPIC, dv_control_target, cb_dv_control_target_conversion)
    pub_dv_control = rospy.Publisher(OUTPUT_CMD_TOPIC,
                          Cmd, queue_size=1)
    rospy.spin()