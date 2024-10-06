#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def Camera_feed():
    rospy.init_node("camera_node", anonymous=True)
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    
    # Try with different camera indexes if needed
    cam_feed = cv2.VideoCapture(2)  # Use 0 for default camera, change to 1 if you have multiple cameras

    if not cam_feed.isOpened():
        rospy.logerr("Failed to open camera")
        return
    
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        ret, frame = cam_feed.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            continue

        # Convert the OpenCV image (BGR) to ROS Image message
        img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        
        # Publish the image to the /camera/image_raw topic
        pub.publish(img_msg)
        
        rate.sleep()

    cam_feed.release()

if __name__ == '__main__':
    try:
        Camera_feed()
    except rospy.ROSInterruptException:
        pass
