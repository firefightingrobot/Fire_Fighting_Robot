#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, String

# Callback function to handle right ultrasonic data
def right_distance_callback(msg):
    rospy.loginfo("Right Ultrasonic Distance: %.2f cm", msg.data)

# Callback function to handle left ultrasonic data
def left_distance_callback(msg):
    rospy.loginfo("Left Ultrasonic Distance: %.2f cm", msg.data)

# Callback function to handle gas sensor data
def gas_sensor_callback(msg):
    rospy.loginfo("Gas Sensor Value: %.2f", msg.data)

# Callback function to handle temperature data
def temperature_callback(msg):
    rospy.loginfo("Temperature: %.2f °C", msg.data)

# Callback function to handle humidity data
def humidity_callback(msg):
    rospy.loginfo("Humidity: %.2f %%", msg.data)

# Callback function to handle flame sensor data
def flame_sensor_callback(msg, sensor_id):
    rospy.loginfo("Flame Sensor %d: %s", sensor_id, msg.data)

def listener():
    # Initialize the ROS node
    rospy.init_node('arduino_sensor_listener', anonymous=True)

    # Subscribers for various sensor topics
    rospy.Subscriber("right_ultrasonic", Float64, right_distance_callback)
    rospy.Subscriber("left_ultrasonic", Float64, left_distance_callback)
    rospy.Subscriber("gas_sensor", Float64, gas_sensor_callback)
    rospy.Subscriber("temperature", Float64, temperature_callback)
    rospy.Subscriber("humidity", Float64, humidity_callback)

    # Subscribers for flame sensors
    for i in range(5):  # As there are 5 flame sensors
        rospy.Subscriber("flame_sensor_{}".format(i), String, flame_sensor_callback, i)

    # Keep the node running and listening for messages
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
