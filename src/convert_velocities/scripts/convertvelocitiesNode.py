#!/usr/bin/env python
# rospy for the subscriber and publisher
import rospy
# type of messages we are going to use:
# this is the command to the motors:
from std_msgs.msg import Float64
# this is the command of velocity from the keyboard node:
from geometry_msgs.msg import Twist
# Miscellaneous libraries that we might use:
import matplotlib.pyplot as plt
import numpy as np

left_front_wheel = 0
right_front_wheel = 0
left_back_wheel = 0
right_back_wheel = 0

def velocity_callback(msg):
    tw = 0.2 #trackwidth of the rover
    x_dot = msg.linear.x
    psi_dot = msg.angular.z

    #Motors' velocities:
    global left_front_wheel, right_front_wheel,left_back_wheel, right_back_wheel
    r = 0.035 #radius of wheels
    left_front_wheel = x_dot - psi_dot*tw/2
    left_front_wheel = left_front_wheel/r
    right_front_wheel = x_dot + psi_dot*tw/2
    right_front_wheel = right_front_wheel/r
    left_back_wheel = x_dot - psi_dot*tw/2
    left_back_wheel = left_back_wheel/r
    right_back_wheel = x_dot + psi_dot*tw/2
    right_back_wheel = right_back_wheel/r
    
    

def main():
    global left_front_wheel, right_front_wheel,left_back_wheel, right_back_wheel
    rospy.init_node('VelocitiesConverter')
    rospy.Subscriber("/cmd_vel", Twist, velocity_callback)
    RW_front_cmd_topic = "/rover/right_front_wheel_velocity_controller/command"
    LW_front_cmd_topic = "/rover/left__front_wheel_velocity_controller/command"
    RW_back_cmd_topic = "/rover/right_back_wheel_velocity_controller/command"
    LW_back_cmd_topic = "/rover/left__back_wheel_velocity_controller/command"
    RW_front_pub = rospy.Publisher(RW_front_cmd_topic, Float64, queue_size = 1)
    LW_front_pub = rospy.Publisher(LW_front_cmd_topic, Float64, queue_size = 1)
    RW_back_pub = rospy.Publisher(RW_back_cmd_topic, Float64, queue_size = 1)
    LW_back_pub = rospy.Publisher(LW_back_cmd_topic, Float64, queue_size = 1)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
	RW_front_pub.publish(right_front_wheel)
        LW_front_pub.publish(left_front_wheel)
        RW_back_pub.publish(right_back_wheel)
        LW_back_pub.publish(left_back_wheel)

	print("Converting keyboard commands to motor commands")
        rate.sleep()

if __name__ == '__main__':
    main()
