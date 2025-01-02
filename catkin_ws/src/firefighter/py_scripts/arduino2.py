#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist

# Callback for receiving right wheel tick count
def right_ticks_callback(msg):
    rospy.loginfo("Right Wheel Ticks: %d", msg.data)

# Callback for receiving left wheel tick count
def left_ticks_callback(msg):
    rospy.loginfo("Left Wheel Ticks: %d", msg.data)

# Callback for receiving pump control message for left pump
def left_pump_callback(msg):
    if msg.data:
        rospy.loginfo("Left Pump: ON")
    else:
        rospy.loginfo("Left Pump: OFF")

# Callback for receiving pump control message for right pump
def right_pump_callback(msg):
    if msg.data:
        rospy.loginfo("Right Pump: ON")
    else:
        rospy.loginfo("Right Pump: OFF")

# Callback for receiving velocity command (Twist message)
def cmd_vel_callback(msg):
    rospy.loginfo("Received Velocity Command: Linear: %f, Angular: %f", msg.linear.x, msg.angular.z)

def main():
    # Initialize the ROS node
    rospy.init_node('arduino_receiver_node', anonymous=True)

    # Subscribe to the relevant topics
    rospy.Subscriber('right_ticks', Int16, right_ticks_callback)
    rospy.Subscriber('left_ticks', Int16, left_ticks_callback)
    rospy.Subscriber('left_pump_control', Bool, left_pump_callback)
    rospy.Subscriber('right_pump_control', Bool, right_pump_callback)
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

    # Spin to keep the node running and processing callbacks
    rospy.spin()

if __name__ == '__main__':
    main()
