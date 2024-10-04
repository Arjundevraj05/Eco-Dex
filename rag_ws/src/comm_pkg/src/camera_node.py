#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def camera_publisher():
    rospy.init_node('camera_node', anonymous=True)
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    bridge = CvBridge()
    cap = cv2.VideoCapture(0)  # Open camera, 0 for default camera

    if not cap.isOpened():
        rospy.logerr("Unable to open camera")
        return

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            continue

        # Convert the OpenCV image to ROS Image message
        img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        
        # Publish the image message
        pub.publish(img_msg)
        
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
