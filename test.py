import cv2
import numpy as np
from ultralytics import YOLO
import time
import serial
import threading

class EmergencyVehicleDetector:
    def __init__(self, model_path='yolov8n.pt', conf_threshold=0.5):
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.class_names = self.model.names
        self.vehicle_classes = [2, 3, 5, 7]
        self.emergency_detected = False
        self.emergency_confidence = 0.0
        self.emergency_vehicle_frames = 0
        self.required_frames = 3
        
    def detect_emergency_vehicle(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        combined_mask = cv2.bitwise_or(red_mask, blue_mask)
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        emergency_light_count = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 50:
                emergency_light_count += 1
        is_emergency = False
        confidence = 0.0
        if emergency_light_count >= 2:
            results = self.model(frame, verbose=False)
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    conf = box.conf.item()
                    cls = int(box.cls.item())
                    if conf > self.conf_threshold and cls in self.vehicle_classes:
                        is_emergency = True
                        confidence = conf
                        break
        if is_emergency:
            self.emergency_vehicle_frames += 1
            if self.emergency_vehicle_frames >= self.required_frames:
                self.emergency_detected = True
                self.emergency_confidence = confidence
        else:
            self.emergency_vehicle_frames = max(0, self.emergency_vehicle_frames - 1)
            if self.emergency_vehicle_frames == 0:
                self.emergency_detected = False
                self.emergency_confidence = 0.0
        return self.emergency_detected, self.emergency_confidence
    
    def detect(self, frame, roi=None):
        if roi is not None:
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            roi_points = np.array(roi, dtype=np.int32)
            cv2.fillPoly(mask, [roi_points], 255)
            masked_frame = cv2.bitwise_and(frame, frame, mask=mask)
            roi_area = cv2.contourArea(roi_points)
        else:
            masked_frame = frame
            roi_area = frame.shape[0] * frame.shape[1]
        emergency_detected, emergency_conf = self.detect_emergency_vehicle(frame)
        results = self.model(masked_frame, verbose=False)
        vehicles = []
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = box.conf.item()
                cls = int(box.cls.item())
                if conf > self.conf_threshold and cls in self.vehicle_classes:
                    vehicles.append([x1, y1, x2, y2, conf, cls])
        if len(vehicles) > 0:
            vehicle_area = sum((x2-x1)*(y2-y1) for x1, y1, x2, y2, _, _ in vehicles)
            density = min(1.0, vehicle_area / (roi_area * 0.5))
        else:
            density = 0.0
        annotated_frame = results[0].plot()
        if roi is not None:
            cv2.polylines(annotated_frame, [roi_points], True, (0, 255, 0), 2)
        cv2.putText(annotated_frame, f"Density: {density:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(annotated_frame, f"Vehicles: {len(vehicles)}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if emergency_detected:
            cv2.putText(annotated_frame, "EMERGENCY VEHICLE DETECTED!", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            if time.time() % 1 < 0.5:
                cv2.rectangle(annotated_frame, (0, 0), 
                              (annotated_frame.shape[1], annotated_frame.shape[0]), 
                              (0, 0, 255), 10)
        return vehicles, len(vehicles), density, emergency_detected, annotated_frame

class ArduinoController:
    def __init__(self, port='COM6', baud_rate=9600):
        self.port = port
        self.baud_rate = baud_rate
        self.serial_conn = None
        self.is_connected = False
    def connect(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2)
            self.is_connected = True
            print(f"Connected to Arduino on {self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect to Arduino: {e}")
            self.is_connected = False
            return False
    def disconnect(self):
        if self.serial_conn and self.is_connected:
            self.serial_conn.close()
            self.is_connected = False
            print("Disconnected from Arduino")
    def send_command(self, command):
        if self.serial_conn and self.is_connected:
            try:
                self.serial_conn.write(f"{command}\n".encode())
                return True
            except Exception as e:
                print(f"Failed to send command: {e}")
                return False
        else:
            print("Not connected to Arduino")
            return False

class TrafficController:
    def __init__(self, video_source=0, arduino_port='COM6'):
        self.video_source = video_source
        self.detector = EmergencyVehicleDetector(conf_threshold=0.4)
        self.arduino = ArduinoController(port=arduino_port)
        self.min_green_time = 10
        self.max_green_time = 60
        self.yellow_time = 3
        self.current_state = "red"
        self.remaining_time = 30
        self.current_density = 0.0
        self.emergency_mode = False
        self.emergency_cooldown = 0
        self.emergency_duration = 30
        self.running = False
        self.traffic_light_thread = None
    def calculate_green_time(self, density):
        if density > 0.8:
            return self.max_green_time
        elif density < 0.2:
            return self.min_green_time
        else:
            return int(self.min_green_time + (self.max_green_time - self.min_green_time) 
                       * ((density - 0.2) / 0.6))
    def traffic_light_cycle(self):
        self.arduino.connect()
        self.current_state = "red"
        self.remaining_time = 30
        self.arduino.send_command("RED")
        while self.running:
            if self.emergency_mode:
                if self.current_state != "green":
                    if self.current_state == "red":
                        self.current_state = "yellow"
                        self.arduino.send_command("YELLOW")
                        time.sleep(2)
                    self.current_state = "green"
                    self.remaining_time = self.emergency_duration
                    self.arduino.send_command("GREEN")
                    print("EMERGENCY: Switching to GREEN for emergency vehicle")
                if self.emergency_cooldown > 0:
                    self.emergency_cooldown -= 1
                    if self.emergency_cooldown == 0:
                        self.emergency_mode = False
                        print("Emergency mode ended")
            else:
                self.remaining_time -= 1
                if self.remaining_time <= 0:
                    if self.current_state == "red":
                        self.current_state = "green"
                        green_time = self.calculate_green_time(self.current_density)
                        self.remaining_time = green_time
                        self.arduino.send_command("GREEN")
                        print(f"Switched to GREEN for {green_time}s (density: {self.current_density:.2f})")
                    elif self.current_state == "green":
                        self.current_state = "yellow"
                        self.remaining_time = self.yellow_time
                        self.arduino.send_command("YELLOW")
                        print("Switched to YELLOW")
                    elif self.current_state == "yellow":
                        self.current_state = "red"
                        self.remaining_time = 30
                        self.arduino.send_command("RED")
                        print("Switched to RED")
            time.sleep(1)
        self.arduino.send_command("RED")
        self.arduino.disconnect()
    def start(self):
        self.running = True
        self.traffic_light_thread = threading.Thread(target=self.traffic_light_cycle)
        self.traffic_light_thread.daemon = True
        self.traffic_light_thread.start()
    def stop(self):
        self.running = False
        if self.traffic_light_thread:
            self.traffic_light_thread.join(timeout=5)
    def activate_emergency_mode(self):
        self.emergency_mode = True
        self.emergency_cooldown = self.emergency_duration
        print("Emergency mode activated - creating green corridor")
    def run(self):
        roi = [(100, 100), (540, 100), (540, 380), (100, 380)]
        self.start()
        cap = cv2.VideoCapture(self.video_source)
        if not cap.isOpened():
            print(f"Error: Could not open video source {self.video_source}")
            self.stop()
            return
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("End of video stream")
                    break
                frame = cv2.resize(frame, (640, 480))
                _, vehicle_count, density, emergency_detected, annotated_frame = self.detector.detect(frame, roi)
                self.current_density = density
                if emergency_detected and not self.emergency_mode:
                    self.activate_emergency_mode()
                light_color = (0, 255, 0) if self.current_state == "green" else \
                              (0, 255, 255) if self.current_state == "yellow" else \
                              (0, 0, 255)
                cv2.circle(annotated_frame, (580, 40), 30, light_color, -1)
                cv2.putText(annotated_frame, f"Time: {self.remaining_time}s", 
                            (500, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(annotated_frame, f"State: {self.current_state.upper()}", 
                            (500, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                if self.emergency_mode:
                    cv2.putText(annotated_frame, "EMERGENCY MODE", 
                                (500, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow("Traffic Management", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
            self.stop()
            print("Traffic controller stopped")

if __name__ == "__main__":
    controller = TrafficController(
        video_source="1.mp4",
        arduino_port="COM6"
    )
    controller.run()