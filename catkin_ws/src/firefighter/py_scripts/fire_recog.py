#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import Int8
from time import sleep

# Initialize the camera
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Width
cap.set(4, 480)  # Height

def process():
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Convert the frame to HSV (Hue, Saturation, Value) color space for better fire detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the HSV ranges for different colors
        # Red color range (low and high)
        lower_red_1 = np.array([0, 120, 70])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 120, 70])  # Upper range of red (for the second red range)
        upper_red_2 = np.array([180, 255, 255])

        # Orange color range
        lower_orange = np.array([10, 150, 150])
        upper_orange = np.array([30, 255, 255])

        # Yellow color range
        lower_yellow = np.array([25, 100, 100])
        upper_yellow = np.array([35, 255, 255])

        # Blue color range (to exclude blue background, if necessary)
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        # Create masks for red, orange, yellow, and blue
        mask_red_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask_red_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)

        mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # Combine all color masks
        fire_mask = cv2.bitwise_or(mask_red, mask_orange)
        fire_mask = cv2.bitwise_or(fire_mask, mask_yellow)
        fire_mask = cv2.bitwise_and(fire_mask, cv2.bitwise_not(mask_blue))  # Exclude blue background

        # OPTIONAL: Display the individual masks for debugging
        cv2.imshow("Red Mask", mask_red)
        cv2.imshow("Orange Mask", mask_orange)
        cv2.imshow("Yellow Mask", mask_yellow)
        cv2.imshow("Blue Mask", mask_blue)

        # Find contours to identify the size and position of detected flames
        contours, _ = cv2.findContours(fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        fire_detected = False
        x_center_of_fire = 0

        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Reduced the threshold for debugging
                fire_detected = True
                x, y, w, h = cv2.boundingRect(contour)
                x_center_of_fire = int(x + w / 2)  # Get the x-center of the detected fire
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)  # Draw rectangle around fire
                print("Fire Detected!")

                # Publish fire detection signal
                fire_pub.publish(1)
                break

        if not fire_detected:
            print("No Fire Detected")
            fire_pub.publish(0)

        # Define tolerance zones for decision-making (left, right, straight)
        rows, columns, _ = frame.shape
        y_tolerance_left = int(columns / 3.0)  # Left zone
        y_tolerance_right = int(columns * 2 / 3.0)  # Right zone
        y_center = int(columns / 2.0)  # Middle zone (straight)

        # Decision logic based on the position of the fire in the frame
        if fire_detected:
            if x_center_of_fire < y_tolerance_left:  # Fire is on the left
                left = 1
                right = 0
                straight = 0
                print("Turning Left")
            elif x_center_of_fire > y_tolerance_right:  # Fire is on the right
                left = 0
                right = 1
                straight = 0
                print("Turning Right")
            else:  # Fire is in the middle
                left = 0
                right = 0
                straight = 1
                print("Moving Straight")

            # Publish the directional commands
            left_pub.publish(left)
            right_pub.publish(right)
            straight_pub.publish(straight)

        # Show the processed frame with fire detection
        cv2.imshow("Frame", frame)

        # Wait for user input to quit (no blocking wait, continuous processing)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break

if __name__ == '__main__':
    rospy.init_node('fire_detector_with_direction', anonymous=True)

    # Publishers for fire detection and movement commands
    fire_pub = rospy.Publisher('fire_detected', Int8, queue_size=10)
    right_pub = rospy.Publisher('turn_right', Int8, queue_size=10)
    left_pub = rospy.Publisher('turn_left', Int8, queue_size=10)
    straight_pub = rospy.Publisher('straight', Int8, queue_size=10)

    # Give the system time to initialize
    sleep(1)

    process()
