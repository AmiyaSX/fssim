#!/usr/bin/env python
import rospy
import math
# ROS Msgs
from fssim_common.msg import dv_control_target
from fssim_common.msg import Cmd
from fssim_common.msg import State

# Use the config variables
INPUT_CMD_TOPIC = rospy.get_param('conversion/topics/origin_cmd_topic') # /navigation/dv_control_target (MARS)
OUTPUT_CMD_TOPIC = rospy.get_param('conversion/topics/target_cmd_topic') # /fssim/cmd (fssim)
INPUT_STATE_TOPIC = rospy.get_param('conversion/topics/origin_odom_topic') # /fssim/base_pose_ground_truth (fssim)

MAX_TARGET_SPEED = rospy.get_param('conversion/limits/max_target_speed') # max_target_speed
MAX_STEERING_ANGLE = rospy.get_param('conversion/limits/max_steering_angle') # max_steering_angle

class ConvertCommands:

    def __init__(self):
        self.kp = 0.5
        self.vehicle_speed = 0
        self.pub_convert_commands = rospy.Publisher(OUTPUT_CMD_TOPIC,
                          Cmd, queue_size=1)

    # get current vehicle speed using velocity in x and y direction
    def cb_get_vehicle_speed(self, car_state):
        vehicle_speed = math.sqrt(car_state.vx**2 + car_state.vy**2)
        if(vehicle_speed > MAX_TARGET_SPEED):
            vehicle_speed = MAX_TARGET_SPEED
        self.vehicle_speed = vehicle_speed

    # map the speed state and steering angle range in -1 ~ 1
    def cb_dv_control_target_conversion(self, dv_control_target):
        msg = Cmd()
        target_speed = dv_control_target.dv_speed_target
        if(dv_control_target.dv_speed_target > MAX_TARGET_SPEED):
            target_speed = MAX_TARGET_SPEED

        p = target_speed - self.vehicle_speed
        dv = self.kp * p
        dv = min(max(dv, -1), 1)
        
        delta = dv_control_target.dv_steering_angle_target / MAX_STEERING_ANGLE # (-1-right-0-left-1)
        if delta > 1: delta = 1
        elif delta < -1: delta = -1
        
        msg.dc = dv
        msg.delta = delta
        self.pub_convert_commands.publish(msg)

# FssimCmdConversion node converts input from dv_control_target msg(MARS) outputs Cmd msg(fssim)
if __name__ == '__main__':
    rospy.init_node('fssim_cmd_conversion_node')
    rospy.loginfo("Fssim_cmd_conversion node started")

    convert_commands = ConvertCommands()
    rospy.Subscriber(INPUT_STATE_TOPIC, State, convert_commands.cb_get_vehicle_speed)
    rospy.Subscriber(INPUT_CMD_TOPIC, dv_control_target, convert_commands.cb_dv_control_target_conversion)
    rospy.spin()