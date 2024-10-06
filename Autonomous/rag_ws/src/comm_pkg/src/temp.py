import cv2
from ultralytics import YOLO  

def get_box_size(box):
    x1, y1, x2, y2 = box.xyxy[0]
    box_width = (x1 - x2)
    box_height = (y1 - y2)
    box_size = (box_width**2 + box_height**2)**0.5
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
        x1, y1, x2, y2 = closest_object.xyxy[0]
        box_info['Coordinates'] = (float(x1), float(y1), float(x2), float(y2))

        if class_name in ["METAL", "GLASS", "PLASTIC"]:
            box_info['isBiodegradable'] = False
        else:
            box_info['isBiodegradable'] = True

        return box_info
    return None

def detect(model, camera, ahead, thres_box_frame_ratio=0.6):
    cap = cv2.VideoCapture(camera)

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_size = (frame_width**2 + frame_height**2)**0.5

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return None

    results = None
    found = False
    while not found :
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not receive frame.")
            break

        results = model(frame, imgsz=512, save=False, plots=False, device='cpu', verbose=False)

        if ahead:
            remove_large(results, frame_size, thres_box_frame_ratio)

        annotated_frame = results[0].plot()
        cv2.imshow("Garbage Detection", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if results[0].boxes is not None:
            found = True

    cap.release()
    cv2.destroyAllWindows()

    if found:
        return get_closest_object_info(results)
    return None

def run_detection(model_path):
    camera = 0  
    model = YOLO(model_path)

    while True:
        try:
            box_info = detect(model, camera, ahead=True)
            print(box_info)
            # Add code here to send box_info to Arduino or move towards object
            box_info = detect(model, camera, ahead=False)
            print(box_info)
            # Add code to pause and collect object
        except:
            break

model_path="runs/detect/final_train/weights/best_openvino_model"
run_detection(model_path)