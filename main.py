
import cv2
from ultralytics import YOLO
import serial
import time
import numpy as np
from collections import deque
from typing import Dict, Tuple, List

# Config
class Config:
    WEIGHTS = r"dronehuman.pt"
    CAMERA = 0
    DISPLAY_W = 720
    TRACK_IMGSZ = 320
    CONF_THRESH = 0.5
    IOU_THRESH = 0.45
    USE_GPU = False
    TARGET_FPS = 30
    DRAW_GRID = True
    FRAME_SKIP = 1
    MIN_WIDTH = 50
    TARGET_CLASSES = ['human', 'drone']
    FPS_WINDOW = 10
    CENTER_W_RATIO = 0.2
    CENTER_H_RATIO = 0.2
    FOCAL_LENGTH_MM = 3.0
    SENSOR_WIDTH_MM = 3.6
    IMAGE_WIDTH_PX = 960
    FOCAL_LENGTH_PX = (FOCAL_LENGTH_MM / SENSOR_WIDTH_MM) * IMAGE_WIDTH_PX
    ARDUINO_PORT = 'COM3'
    BAUD_RATE = 115200
    DETECTION_THRESHOLD = 5
    NO_OBJECT_THRESHOLD = 5
    POSITION_THRESHOLD = 0.1

# Initialize zone polygons
def init_zones(frame_shape: Tuple[int, int], center_w_ratio: float = Config.CENTER_W_RATIO, 
               center_h_ratio: float = Config.CENTER_H_RATIO) -> Dict[str, np.ndarray]:
    h, w = frame_shape
    cx, cy = w // 2, h // 2
    hw = int(w * center_w_ratio) // 2
    hh = int(h * center_h_ratio) // 2
    return {
        "C": np.array([[cx-hw, cy-hh], [cx+hw, cy-hh], [cx+hw, cy+hh], [cx-hw, cy+hh]], np.int32),
        "T": np.array([[0,0], [w-1,0], [cx+hw, cy-hh], [cx-hw, cy-hh]], np.int32),
        "B": np.array([[0,h-1], [w-1,h-1], [cx+hw, cy+hh], [cx-hw, cy+hh]], np.int32),
        "L": np.array([[0,0], [0,h-1], [cx-hw, cy+hh], [cx-hw, cy-hh]], np.int32),
        "R": np.array([[w-1,0], [w-1,h-1], [cx+hw, cy+hh], [cx+hw, cy-hh]], np.int32)
    }

# Draw zones and labels
def draw_zones(frame: np.ndarray, zones: Dict[str, np.ndarray], zone_labels: Dict[str, Tuple[int, int]]) -> np.ndarray:
    if not Config.DRAW_GRID:
        return frame
    for poly in zones.values():
        cv2.polylines(frame, [poly], True, (0, 0, 0), 2)
    font_scale = min(frame.shape[:2]) / 1000.0
    for label, (x, y) in zone_labels.items():
        cv2.putText(frame, label, (x, y), cv2.FONT_HERSHEY_DUPLEX, font_scale, (0, 0, 0), 2)
    return frame

# Determine object region
def get_object_region(bbox: Tuple[int, int, int, int], zones: Dict[str, np.ndarray]) -> str:
    x1, y1, x2, y2 = bbox
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    for name, poly in zones.items():
        if cv2.pointPolygonTest(poly, (cx, cy), False) >= 0:
            return name
    return "C"

# Read distance from Arduino
def get_distance_from_arduino(arduino: serial.Serial) -> float:
    max_attempts = 5
    for _ in range(max_attempts):
        try:
            arduino.write(b"GET\n")
            time.sleep(0.05)
            if arduino.in_waiting > 0:
                distance = arduino.readline().decode('utf-8').strip()
                if distance != "E" and distance.isdigit():
                    dist = float(distance)
                    if 2 <= dist <= 500:
                        return dist
        except Exception as e:
            print(f"Error reading from Arduino: {e}")
        time.sleep(0.02)
    return None

# Initialize zone labels
def init_zone_labels(frame_shape: Tuple[int, int]) -> Dict[str, Tuple[int, int]]:
    h, w = frame_shape
    cx, cy = w // 2, h // 2
    hw = int(w * Config.CENTER_W_RATIO) // 2
    hh = int(h * Config.CENTER_H_RATIO) // 2
    return {
        "CENTER": (cx-40, cy+5),
        "TOP": (cx-20, cy-hh-20),
        "BOTTOM": (cx-40, cy+hh+30),
        "LEFT": (cx-hw-70, cy+5),
        "RIGHT": (cx+hw+20, cy+5)
    }

# Main function
def main():
    # Connect to Arduino
    try:
        arduino = serial.Serial(Config.ARDUINO_PORT, Config.BAUD_RATE, timeout=1)
        time.sleep(2)
    except serial.SerialException as e:
        print(f"Error connecting to Arduino: {e}")
        return

    # Load YOLO model
    try:
        model = YOLO(Config.WEIGHTS)
    except Exception as e:
        print(f"Error: Failed to load YOLO model: {e}")
        return

    # Initialize camera
    cap = cv2.VideoCapture(Config.CAMERA)
    cap.set(3, 1080)
    cap.set(4, 720)
    if not cap.isOpened():
        print("Error: Could not open video file.")
        return

    width, height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    display_height = int(Config.DISPLAY_W * height / width)
    print(f"Original video resolution: {width}x{height}")
    print(f"Display resolution: {Config.DISPLAY_W}x{display_height}")
    print("🔍 Obstacle Avoidance and Drone Detection Started. Press 'q' to quit.")

    # Initialize variables
    target_class_ids = {name.lower(): i for i, name in model.model.names.items() if name.lower() in Config.TARGET_CLASSES}
    zones = init_zones((display_height, Config.DISPLAY_W))
    zone_labels = init_zone_labels((display_height, Config.DISPLAY_W))
    frame_times = deque(maxlen=Config.FPS_WINDOW)
    prev_time = time.time()
    frame_count = 0
    last_distance_cm = None
    distance_frame_count = 0
    DISTANCE_UPDATE_INTERVAL = 5
    last_direction = None
    tracking_start_time = None
    last_class = None
    last_command_sent = None
    last_detection_time = None
    last_norm_x = None
    last_norm_y = None
    no_object_counter = 0
    detection_counter = 0

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("❌ Camera error.")
            break

        frame_count += 1
        if frame_count % Config.FRAME_SKIP != 0:
            continue

        frame = cv2.resize(frame, (Config.DISPLAY_W, display_height), interpolation=cv2.INTER_LINEAR)
        results = model(frame, verbose=False, imgsz=Config.TRACK_IMGSZ, conf=Config.CONF_THRESH, iou=Config.IOU_THRESH)[0]
        overlay_frame = draw_zones(frame, zones, zone_labels)
        object_detected = False
        best_conf = 0
        best_cmd = None
        best_action = None
        best_norm_x = None
        best_norm_y = None
        best_class_name = None

        # Update distance
        if distance_frame_count % DISTANCE_UPDATE_INTERVAL == 0:
            distance_cm = get_distance_from_arduino(arduino)
            if distance_cm is not None:
                last_distance_cm = distance_cm
        else:
            distance_cm = last_distance_cm
        distance_frame_count += 1

        for box in results.boxes:
            class_id = int(box.cls[0])
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            w = x2 - x1
            class_name = model.model.names[class_id].lower()

            if class_name in Config.TARGET_CLASSES and conf > Config.CONF_THRESH and w > Config.MIN_WIDTH:
                object_detected = True
                region = get_object_region((x1, y1, x2, y2), zones)
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # Draw bounding box
                label = f"{class_name} {conf:.2f}"
                color = (0, 0, 255) if class_name == 'human' else (0, 255, 0)
                cv2.rectangle(overlay_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(overlay_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

                # Normalized coordinates
                norm_x = center_x / Config.DISPLAY_W - 0.5
                norm_y = center_y / display_height - 0.5

                # Action and direction
                direction_cmd = region
                if class_name == 'drone':
                    action_type = 'F'
                elif class_name == 'human':
                    action_type = 'A'
                    if direction_cmd != 'C':
                        direction_cmd = {'L': 'R', 'R': 'L', 'T': 'B', 'B': 'T'}.get(direction_cmd, direction_cmd)

                # Calculate actual width and height
                pixel_width = w
                pixel_height = y2 - y1
                actual_width_cm = (pixel_width * distance_cm) / Config.FOCAL_LENGTH_PX if distance_cm and distance_cm > 0 else None
                actual_height_cm = (pixel_height * distance_cm) / Config.FOCAL_LENGTH_PX if distance_cm and distance_cm > 0 else None

                # Display distance, width, and height
                text_y = y1 - 40
                if distance_cm:
                    cv2.putText(overlay_frame, f"Dist: {distance_cm:.1f} cm", (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                    text_y -= 20
                if actual_width_cm and actual_height_cm:
                    cv2.putText(overlay_frame, f"W: {actual_width_cm:.1f} cm", (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    text_y -= 20
                    cv2.putText(overlay_frame, f"H: {actual_height_cm:.1f} cm", (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                if conf > best_conf:
                    best_conf = conf
                    best_cmd = direction_cmd
                    best_action = action_type
                    best_norm_x = norm_x
                    best_norm_y = norm_y
                    best_class_name = class_name

        # Check position change
        position_changed = False
        if object_detected and last_norm_x is not None and last_norm_y is not None:
            if abs(best_norm_x - last_norm_x) >= Config.POSITION_THRESHOLD or abs(best_norm_y - last_norm_y) >= Config.POSITION_THRESHOLD:
                position_changed = True

        # Send commands to Arduino
        if object_detected and best_cmd:
            detection_counter += 1
            if detection_counter >= Config.DETECTION_THRESHOLD:
                current_direction = f"{best_action}{best_cmd}"
                if last_direction is None or best_cmd != last_direction or position_changed:
                    print(f"📌 {'Following' if best_action == 'F' else 'Avoiding'} {best_class_name}: {best_cmd}")
                    if best_action == 'F':
                        cmd = f"F{best_cmd},0\n"
                        if cmd != last_command_sent:
                            arduino.write(cmd.encode())
                            print(f"📤 Sent: {cmd.strip()}")
                            last_command_sent = cmd
                    elif best_action == 'A':
                        cmd = f"A{best_cmd},0\n"
                        if cmd != last_command_sent:
                            arduino.write(cmd.encode())
                            print(f"📤 Sent: {cmd.strip()}")
                            last_command_sent = cmd
                        if tracking_start_time is None:
                            tracking_start_time = time.time()
                    last_direction = best_cmd
                    last_class = best_class_name
                    last_norm_x = best_norm_x
                    last_norm_y = best_norm_y
                    last_detection_time = time.time()
                    no_object_counter = 0
        else:
            detection_counter = 0
            no_object_counter += 1
            if last_class == 'human' and no_object_counter >= Config.NO_OBJECT_THRESHOLD and tracking_start_time is not None:
                duration = int(time.time() - tracking_start_time)
                cmd = f"A{last_direction},{duration}\n"
                if cmd != last_command_sent:
                    arduino.write(cmd.encode())
                    print(f"📤 Sent: {cmd.strip()}")
                    last_command_sent = cmd
                last_direction = None
                tracking_start_time = None
                last_class = None
                last_norm_x = None
                last_norm_y = None
                last_detection_time = None
                no_object_counter = 0
            elif last_class == 'drone' and no_object_counter >= Config.NO_OBJECT_THRESHOLD:
                cmd = "C\n"
                if cmd != last_command_sent:
                    arduino.write(cmd.encode())
                    print(f"📤 Sent: {cmd.strip()}")
                    last_command_sent = cmd
                last_direction = None
                last_class = None
                last_norm_x = None
                last_norm_y = None
                last_detection_time = None
                no_object_counter = 0

        # Draw crosshair
        h, w = overlay_frame.shape[:2]
        cv2.line(overlay_frame, (w//2, h//2-10), (w//2, h//2+10), (255, 255, 255), 1)
        cv2.line(overlay_frame, (w//2-10, h//2), (w//2+10, h//2), (255, 255, 255), 1)

        # Calculate and display FPS
        current_time = time.time()
        frame_times.append(current_time - prev_time)
        prev_time = current_time
        fps = len(frame_times) / sum(frame_times) if frame_times else 0
        cv2.putText(overlay_frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        # Status text
        status = f"{'Following' if best_action == 'F' else 'Avoiding'}: {best_cmd if best_action == 'F' else {'L': 'R', 'R': 'L', 'T': 'B', 'B': 'T', 'C': 'C'}.get(best_cmd, best_cmd) if best_cmd and best_cmd != 'C' else best_cmd if best_cmd else 'No Object'}" if object_detected else "No Object"
        color = (0, 255, 0) if not object_detected else (0, 0, 255) if best_action == 'A' else (0, 255, 0)
        cv2.putText(overlay_frame, status, (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

        cv2.imshow("Drone and Human Detection", overlay_frame)
        if cv2.waitKey(int(1000 / Config.TARGET_FPS)) & 0xFF == ord('q'):
            arduino.write(b"C\n")
            break

    cap.release()
    cv2.destroyAllWindows()
    arduino.close()

if __name__ == "__main__":
    main()
