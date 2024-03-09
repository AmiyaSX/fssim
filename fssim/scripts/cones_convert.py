#!/usr/bin/env python
import rospy

# ROS Msgs
from fssim_common.msg import Track
from fssim_common.msg import Cone
from fssim_common.msg import Cones

# Use the config variables
INPUT_TRACK_TOPIC = rospy.get_param('conversion/topics/origin_track_topic') # /fssim/track (fssim)
OUTPUT_CONES_TOPIC = rospy.get_param('conversion/topics/target_cones_topic') # /perception/cones (MARS)

class ConvertCones:

    def __init__(self):
        self.pub_convert_cones = rospy.Publisher(OUTPUT_CONES_TOPIC,
                          Cones, queue_size=1)

    # map cones info in fssim to MARS
    def cb_cones_conversion(self, track):
        fs_cones = Cones()
        fs_cones.header.frame_id = "fssim_map"

        for cone in track.cones_left:
            fs_cone = Cone()
            fs_cone.x = cone.x
            fs_cone.y = cone.y
            fs_cone.color = 2
            fs_cone.covariance = [0.005, 0, 0, 0.005]
            
            fs_cones.cones.append(fs_cone)


        for cone in track.cones_right:
            fs_cone = Cone() 
            fs_cone.x = cone.x
            fs_cone.y = cone.y
            fs_cone.color = 1  
            fs_cone.covariance = [0.005, 0, 0, 0.005]

            fs_cones.cones.append(fs_cone)


        for cone in track.cones_orange:
            fs_cone = Cone()
            fs_cone.x = cone.x
            fs_cone.y = cone.y
            fs_cone.color = 3
            fs_cone.covariance = [0.005, 0, 0, 0.005]

            fs_cones.cones.append(fs_cone)

        for cone in track.cones_orange_big:
            fs_cone = Cone()
            fs_cone.x = cone.x
            fs_cone.y = cone.y
            fs_cone.color = 4
            fs_cone.covariance = [0.005, 0, 0, 0.005]

            fs_cones.cones.append(fs_cone)

        self.pub_convert_cones.publish(fs_cones)

# FssimConesConversion node converts input from Track msg(fssim) outputs Cones msg(MARS)
if __name__ == '__main__':
    rospy.init_node('fssim_cones_conversion_node')
    rospy.loginfo("Fssim_cones_conversion node started")

    convert_cones = ConvertCones()
    rospy.Subscriber(INPUT_TRACK_TOPIC, Track, convert_cones.cb_cones_conversion)
    rospy.spin()