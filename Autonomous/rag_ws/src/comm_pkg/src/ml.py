#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import json
from ultralytics import YOLO  
import cv2

global frame_height, frame_width, model_path
global ahead
ahead = 'true'

bridge = CvBridge()

def get_box_size(box):
    x1, y1, x2, y2 = box.xyxy[0]
    box_width = abs(x2 - x1)
    box_height = abs(y2 - y1)
    box_size = (box_width*2 + box_height)*0.5
    return box_size

def remove_large(results, frame_size, thres_box_frame_ratio=0.6):
    keep_boxes = []
    for i, box in enumerate(results[0].boxes):
        box_size = get_box_size(box)
        box_frame_ratio = box_size / frame_size

        if box_frame_ratio <= thres_box_frame_ratio:
            keep_boxes.append(i)

    if keep_boxes:
        results[0].boxes = results[0].boxes[keep_boxes]
        get_closest_object_info(results)
    else:
        results[0].boxes = None

def display_box_details(results):
    if results[0].boxes is not None:
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            cls_id = int(box.cls[0])
            confidence = box.conf[0]
            class_name = results[0].names[cls_id]
            print(f"Class: {class_name}, Confidence: {confidence:.2f}, Box: [{x1}, {y1}, {x2}, {y2}]")

def get_closest_object_info(results):
    if results[0].boxes is not None:
        closest_object = results[0].boxes[0]
        for box in results[0].boxes:
            if get_box_size(box) > get_box_size(closest_object):
                closest_object = box

        box_info = {}
        cls_id = int(closest_object.cls[0])
        class_name = results[0].names[cls_id]
        box_info['Class'] = class_name
        x1, _, x2, _ = closest_object.xyxy[0]
        box_info['Centre'] = (frame_width*0.5, frame_height*0.5)
        box_info['Object_pos'] = ((float(x1)+float(x2)/2), (frame_height/2))
        box_info['Ahead'] = ahead

        if class_name in ["METAL", "GLASS", "PLASTIC"]:
            box_info['isBiodegradable'] = False
        else:
            box_info['isBiodegradable'] = True

        move_publisher(box_info)
    return None

def image_callback(ros_image, model, thres_box_frame_ratio=0.6):
    global bridge
    global frame_height, frame_width
    global ahead
    
    try:
        # Convert the ROS image message to an OpenCV image using cv_bridge
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")

        # Get frame size for object size comparisons
        frame_height, frame_width, _ = frame.shape
        frame_size = (frame_width * 2 + frame_height) * 0.5

        results = model(frame, imgsz=512, save=False, plots=False, device='cpu', verbose=False)

        if ahead == 'true':
            remove_large(results, frame_size, thres_box_frame_ratio)

        annotated_frame = results[0].plot()
        cv2.imshow("Garbage Detection", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown()
            cv2.destroyAllWindows()

        if results[0].boxes is None:
            move_publisher({ahead: ahead})

        if results[0].boxes is not None:
            ahead = 'detected'
            box_info = get_closest_object_info(results)
            print(box_info)  # You can also send this info to another system
            return box_info
        return None

    except CvBridgeError as e:
        print(f"CV Bridge error: {e}")
        return None

def run_detection(model_path):
    # Load the YOLO model
    model = YOLO(model_path)
    # Subscribe to the camera topic (change '/camera/rgb/image_raw' to your topic)
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback, callback_args=(model, True))

    # Keep the node running
    rospy.spin()

def move_publisher(box_info):
    box_info = json.dumps(box_info)
    move_pub = rospy.Publisher('detect', String, queue_size=10)
    move_pub.publish(box_info)
    move_subscriber()


def move_callback(ahead_data):
    global model_path
    global ahead
    if ahead_data.data == 'false':
        ahead = ahead_data.data
        rospy.signal_shutdown()
        run_detection(model_path)

    if ahead_data.data == 'true':
        ahead = ahead_data.data
        rospy.signal_shutdown()
        run_detection(model_path)

    return

def move_subscriber():
    rospy.Subscriber('move', String, move_callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ml_node', anonymous=True)
    model_path = "runs/detect/final_train/weights/best_openvino_model"
    run_detection(model_path)