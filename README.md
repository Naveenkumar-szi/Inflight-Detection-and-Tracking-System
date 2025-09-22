Here’s a clean **README.md** content for your project in Markdown format, aligned with your Python + Arduino system:

# Real-Time Drone and Human Detection with Obstacle Avoidance

## Description
This project implements a **real-time computer vision system** for detecting drones and humans, integrated with **ultrasonic sensing** and **Arduino-controlled servo actuation**. The system can:

- **Follow drones** for monitoring purposes.
- **Avoid humans** for safety and privacy considerations.

It uses YOLOv8 for object detection and zone-based logic for obstacle avoidance actions.

---

## Features
- Real-time detection using YOLOv8 (`dronehuman.pt`)
- Distance measurement via HC-SR04 ultrasonic sensor
- Arduino-controlled servo actuation (PCA9685 driver)
- Zone-based decision making: **Center, Left, Right, Top, Bottom**
- Action modes:
  - `F` = Follow (drones)
  - `A` = Avoid (humans)
- Displays object distance and estimated size (width, height in cm)
- FPS display for performance monitoring

---

## Requirements

### Python Dependencies
- Python 3.9+
- OpenCV (`opencv-python`)
- Ultralytics YOLO (`ultralytics`)
- NumPy (`numpy`)
- pySerial (`pyserial`)

Install dependencies via pip:

```bash
pip install opencv-python ultralytics numpy pyserial
````

### Hardware

* Arduino with PCA9685 servo driver
* Servos for directional control
* HC-SR04 ultrasonic sensor
* Optional: mid-tier GPU for real-time YOLO inference

---

## Usage

1. Connect the Arduino and ensure the **COM port** matches `Config.ARDUINO_PORT`.
2. Upload the `ardiunofile.ino` to your Arduino.
3. Place the `dronehuman.pt` YOLOv8 model in the project directory.
4. Run the Python script:

```bash
python main.py
```

5. Press `q` to quit the program.
6. The system will automatically send **Follow (F)** or **Avoid (A)** commands to Arduino based on detected objects and zones.

---

## File Structure

```
project/
├─ main.py             # Python detection and control script
├─ ardiunofile.ino     # Arduino servo and ultrasonic code
├─ dronehuman.pt       # YOLOv8 model for drone and human detection
├─ README.md           # Project documentation
└─ assets/             # Optional images, flowcharts, or diagrams
```

---

## Zones

The frame is divided into five zones for directional control:

* **Center (C)**
* **Left (L)**
* **Right (R)**
* **Top (T)**
* **Bottom (B)**

Actions are decided based on which zone the object centroid falls into.

---

## References

1. [YOLOv8 Documentation - Ultralytics](https://docs.ultralytics.com)
2. Redmon, J., & Farhadi, A. (2018). *YOLOv3: An Incremental Improvement*. arXiv:1804.02767.
3. [Arduino Documentation](https://docs.arduino.cc)
4. [Adafruit PCA9685 PWM Driver](https://learn.adafruit.com/16-channel-pwm-servo-driver)
5. [HC-SR04 Ultrasonic Sensor Specs](https://www.sparkfun.com/products/15569)

---

## Future Work

* Multi-object tracking
* Reinforcement learning for adaptive servo control
* Improved drone detection dataset for higher robustness

---

## License

This project is licensed under the MIT License.

```

---

If you want, I can also **add badges, setup instructions, and example output screenshots** to make this README **more professional and GitHub-ready**.  

Do you want me to do that next?
```
