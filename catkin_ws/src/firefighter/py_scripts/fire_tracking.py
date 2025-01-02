#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
import numpy as np

# Global variables
turn_right = 0
turn_left = 0
straight = 0
regions = None
fire_detected = False  # New variable to track fire detection status


def clbk_laser(msg):
    """Callback for laser scan data"""
    global regions
    ranges = [r if not np.isnan(r) else 10 for r in msg.ranges]
    regions = {
        'front_L': min(min(ranges[0:130]), 10),
        'fleft': min(min(ranges[131:230]), 10),
        'left': min(min(ranges[231:280]), 10),
        'right': min(min(ranges[571:620]), 10),
        'fright': min(min(ranges[621:720]), 10),
        'front_R': min(min(ranges[721:850]), 10)
    }


def clbk_right(msg):
    """Callback for right turn command"""
    global turn_right, fire_detected
    turn_right = msg.data
    fire_detected = msg.data == 1


def clbk_left(msg):
    """Callback for left turn command"""
    global turn_left, fire_detected
    turn_left = msg.data
    fire_detected = msg.data == 1


def clbk_straight(msg):
    """Callback for straight movement command"""
    global straight, fire_detected
    straight = msg.data
    fire_detected = msg.data == 1


def move_straight():
    """Move straight at constant speed"""
    cmd = Twist()
    cmd.linear.x = 0.5
    cmd.angular.z = 0.0
    pub.publish(cmd)


def turn_right_velo():
    """Turn right at constant speed"""
    cmd = Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = -0.5
    pub.publish(cmd)


def turn_left_velo():
    """Turn left at constant speed"""
    cmd = Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 0.5
    pub.publish(cmd)


def stop():
    """Stop the robot"""
    cmd = Twist()
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)


def main():
    global pub

    rospy.init_node('fire_tracking_node')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.Subscriber('turn_right', Int8, clbk_right)
    rospy.Subscriber('turn_left', Int8, clbk_left)
    rospy.Subscriber('straight', Int8, clbk_straight)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if regions is None:
            rospy.logwarn("Waiting for laser data...")
            rate.sleep()
            continue

        # First check if fire is detected
        if not fire_detected:
            stop()
            rospy.loginfo("No fire detected - stopping")
            rate.sleep()
            continue

        # Check if any region has an obstacle closer than 0.20 meters
        if any(region < 0.20 for region in regions.values()):
            stop()
            rospy.loginfo("Obstacle detected in one or more regions - stopping")
            rate.sleep()
            continue

        # If fire is detected and path is clear, perform movement commands
        if turn_right == 1 and turn_left == 0:
            turn_right_velo()
            rospy.loginfo("Fire detected - turning right")
        elif turn_left == 1 and turn_right == 0:
            turn_left_velo()
            rospy.loginfo("Fire detected - turning left")
        elif straight == 1:
            move_straight()
            rospy.loginfo("Fire detected - moving straight")
        else:
            stop()
            rospy.loginfo("Fire detected but no movement command")

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
