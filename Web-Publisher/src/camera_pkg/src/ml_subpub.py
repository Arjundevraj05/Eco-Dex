#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import json

class GarbageDetector:
    def __init__(self, model_path, camera_topic):
        self.model = YOLO(model_path)
        self.camera_topic = camera_topic
        self.thres_box_frame_ratio = 0.6
        self.results = []
        self.frame_size = 1
        self.bridge = CvBridge()

        # ROS node initialization
        rospy.init_node('garbage_detector_node', anonymous=True)

        # ROS Publisher to publish box info as JSON string
        self.box_info_pub = rospy.Publisher('/garbage_detection/box_info', String, queue_size=10)

        # ROS Subscriber to subscribe to camera topic
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback)

    def image_callback(self, img_msg):
        # Convert ROS Image message to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        frame_height, frame_width, _ = frame.shape
        self.frame_size = (frame_width ** 2 + frame_height ** 2) ** 0.5

        # Perform object detection
        self.results = self.model(frame, verbose=False)
        self.remove_large()

        # Get the closest object information and publish it
        box_info = self.get_closest_object_info()
        if box_info:
            # Convert dictionary to JSON string
            box_info_json = json.dumps(box_info)
            self.box_info_pub.publish(box_info_json)

        # Display annotated frame
        annotated_frame = self.results[0].plot()
        cv2.imshow("Garbage Detection", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown('User exit')

    def remove_large(self):
        keep_boxes = []
        for i, box in enumerate(self.results[0].boxes):
            box_size = self.get_box_size(box)
            box_frame_ratio = box_size / self.frame_size
            if box_frame_ratio <= self.thres_box_frame_ratio:
                keep_boxes.append(i)
        if keep_boxes:
            self.results[0].boxes = self.results[0].boxes[keep_boxes]
        else:
            self.results[0].boxes = None

    def get_box_size(self, box):
        x1, y1, x2, y2 = box.xyxy[0]
        box_width = x2 - x1
        box_height = y2 - y1
        box_size = (box_width ** 2 + box_height ** 2) ** 0.5
        return box_size

    def get_closest_object_info(self):
        if self.results[0].boxes is not None:
            closest_object = self.results[0].boxes[0]
            for box in self.results[0].boxes:
                if self.get_box_size(box) > self.get_box_size(closest_object):
                    closest_object = box
            # Creating the object info dictionary
            box_info = {}
            cls_id = int(closest_object.cls[0])
            class_name = self.results[0].names[cls_id]
            box_info['Class'] = class_name
            x1, y1, x2, y2 = closest_object.xyxy[0]
            #box_info['Coordinates'] = (float(x1), float(y1), float(x2), float(y2))

            # Check if the object is biodegradable
            if class_name in ["METAL", "GLASS", "PLASTIC"]:
                box_info['isBiodegradable'] = False
            else:
                box_info['isBiodegradable'] = True

            return box_info
        return None

    def run(self):
        rospy.spin()  # Keeps the node running

if __name__ == '__main__':
    try:
        # Initialize the GarbageDetector with model path and camera topic
        detector = GarbageDetector("runs/detect/final_train/weights/best.pt", "/camera/image_raw")
        detector.run()
    except rospy.ROSInterruptException:
        pass
