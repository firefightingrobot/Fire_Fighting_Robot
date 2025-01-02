#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import Int8
from time import sleep
import os
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score

# Initialize the camera
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Width
cap.set(4, 480)  # Height

# Folder paths for fire and no-fire images
fire_images_path = '/home/arjuna/catkin_ws/src/firefighter/py_scripts/images/fire/'
no_fire_images_path = '/home/arjuna/catkin_ws/src/firefighter/py_scripts/images/no_fire/'

# Load and preprocess images for manual ML training
def extract_features(image):
    # Convert to grayscale for texture/edge detection or use color histograms for feature extraction
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    resized_image = cv2.resize(gray, (64, 64))  # Resize image to 64x64
    
    # Extract histogram features from the resized image
    hist = cv2.calcHist([resized_image], [0], None, [256], [0, 256])
    hist = cv2.normalize(hist, hist).flatten()  # Normalize and flatten the histogram
    return hist

def load_images(image_paths, label):
    images = []
    labels = []
    for image_path in image_paths:
        img = cv2.imread(image_path)
        features = extract_features(img)  # Extract features from each image
        images.append(features)
        labels.append(label)
    return images, labels

def prepare_data():
    fire_images = [os.path.join(fire_images_path, img) for img in os.listdir(fire_images_path)]
    no_fire_images = [os.path.join(no_fire_images_path, img) for img in os.listdir(no_fire_images_path)]

    # Load and preprocess images
    fire_data, fire_labels = load_images(fire_images, 1)  # Label 1 for fire
    no_fire_data, no_fire_labels = load_images(no_fire_images, 0)  # Label 0 for no fire

    # Combine fire and no fire data
    images = fire_data + no_fire_data
    labels = fire_labels + no_fire_labels

    return images, labels

def train_classifier(images, labels):
    # Split data into train and test sets
    X_train, X_test, y_train, y_test = train_test_split(images, labels, test_size=0.3, random_state=42)

    # Train the classifier (using RandomForest here)
    classifier = RandomForestClassifier(n_estimators=100, random_state=42)
    classifier.fit(X_train, y_train)

    # Predict and evaluate the classifier
    y_pred = classifier.predict(X_test)
    accuracy = accuracy_score(y_test, y_pred)
    print("Model accuracy: {:.2f}%".format(accuracy * 100))

    return classifier

def detect_fire_in_frame(frame, classifier, threshold=0.8):
    """ Use the classifier to detect fire in a given frame with a confidence threshold """
    features = extract_features(frame)  # Extract features from the frame
    prediction_probabilities = classifier.predict_proba([features])  # Get probability for each class
    fire_probability = prediction_probabilities[0][1]  # Probability for fire (class 1)

    print("Fire Probability: {:.2f}%".format(fire_probability * 100))

    if fire_probability >= threshold:
        return 1  # Fire detected
    else:
        return 0  # No fire detected

# Fire detection using color-based thresholding (your existing method)
def process():
    # Load and prepare the dataset for manual ML model
    images, labels = prepare_data()

    # Train a classifier
    classifier = train_classifier(images, labels)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Fire detection using color-based thresholding
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the HSV ranges for different colors
        lower_red_1 = np.array([0, 120, 70])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 120, 70])
        upper_red_2 = np.array([180, 255, 255])
        lower_orange = np.array([10, 150, 150])
        upper_orange = np.array([30, 255, 255])
        lower_yellow = np.array([25, 100, 100])
        upper_yellow = np.array([35, 255, 255])
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

        # Find contours to identify the size and position of detected flames
        contours, _ = cv2.findContours(fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        fire_detected_color_based = False
        x_center_of_fire = 0

        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Adjust this threshold for your use case
                fire_detected_color_based = True
                x, y, w, h = cv2.boundingRect(contour)
                x_center_of_fire = int(x + w / 2)  # Get the x-center of the detected fire
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)  # Draw rectangle around fire
                print("Fire Detected (Color-based)!")

                # Publish fire detection signal (color-based)
                fire_pub.publish(1)
                break

        # Detect fire using manual ML model
        fire_detected_ml = detect_fire_in_frame(frame, classifier)
        print("Fire Detected (ML-based): {}".format(fire_detected_ml))

        if fire_detected_ml:
            print("Fire Detected (ML-based)!")
            fire_pub.publish(1)
        elif not fire_detected_color_based:
            print("No Fire Detected")
            fire_pub.publish(0)

        # Define tolerance zones for decision-making (left, right, straight)
        rows, columns, _ = frame.shape
        y_tolerance_left = int(columns / 3.0)  # Left zone
        y_tolerance_right = int(columns * 2 / 3.0)  # Right zone
        y_center = int(columns / 2.0)  # Middle zone (straight)

        # Decision logic based on the position of the fire in the frame
        if fire_detected_color_based or fire_detected_ml:
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

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break

if __name__ == '__main__':
    rospy.init_node('fire_detector_with_direction_and_ml', anonymous=True)

    # Publishers for fire detection and movement commands
    fire_pub = rospy.Publisher('fire_detected', Int8, queue_size=10)
    right_pub = rospy.Publisher('turn_right', Int8, queue_size=10)
    left_pub = rospy.Publisher('turn_left', Int8, queue_size=10)
    straight_pub = rospy.Publisher('straight', Int8, queue_size=10)

    # Give the system time to initialize
    sleep(1)

    process()
