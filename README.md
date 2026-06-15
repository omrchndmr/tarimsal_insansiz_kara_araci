# Tarımsal İnsansız Kara Aracı
https://github.com/user-attachments/assets/ba17b442-275d-4cad-912a-9253f446445d

**Autonomous Agricultural Ground Vehicle** — a two-computer ROS1 system for
autonomous crop-row following and mechanical weeding.











---

## Short Description

This project implements an autonomous ground vehicle designed for agricultural
row-following and precision hoeing. A **PC station** handles real-time YOLOv8-based
crop detection and steering-angle computation. A **Raspberry Pi** executes motor
control, hoe actuation, and encoder-based row-length counting.

Both platforms communicate over a shared ROS1 (Noetic) network.

---

## Features

- Real-time crop-plant detection with YOLOv8 at ~5 Hz
- Steering-angle computation from dual-plant midpoint geometry
- Differential PWM motor control for differential-drive chassis
- Hoe raise/lower automation via ROS topic triggers
- MZ80 optical-encoder row-length counting
- BNO055 gyroscope yaw data published over serial–ROS bridge
- Sonar-based obstacle sensing (pymata4 / Firmata)
- Flex-sensor stop signal integration
- Modular ROS launch files for PC and Raspberry Pi separately

---

## Tech Stack

| Layer              | Technology                                  |
|--------------------|---------------------------------------------|
| Middleware         | ROS1 Noetic                                 |
| PC Language        | Python 3.8+                                 |
| RPi Language       | Python 3.8+                                 |
| Computer Vision    | OpenCV 4, Ultralytics YOLOv8                |
| Hardware Interface | RPi.GPIO, pymata4 (Firmata), pyserial       |
| Build System       | catkin                                      |
| OS (PC)            | Ubuntu 20.04                                |
| OS (RPi)           | Raspberry Pi OS (64-bit, Bullseye)          |

---

## Project Structure

```
tarimsal_insansiz_kara_araci/
├── pc_station/                   # Code running on the operator PC
│   └── ros_ws/
│       └── src/
│           └── otonom/           # ROS package: vision + sensor bridge
│               ├── scripts/
│               │   ├── otonom.py       # YOLOv8 plant detection & angle publisher
│               │   ├── aci.py          # Gyroscope serial bridge (BNO055)
│               │   ├── flexveri.py     # Flex sensor bridge (pymata4)
│               │   └── mesafeveri.py   # Arduino distance bridge (serial)
│               ├── launch/
│               │   ├── veribasla.launch   # Start all PC-side nodes
│               │   ├── base_link.launch   # TF static transforms
│               │   └── face_detection.launch  # move_base / navigation
│               ├── config/
│               │   ├── global_costmap_params.yaml
│               │   └── local_costmap_params.yaml
│               ├── msg/Num.msg
│               ├── srv/AddTwoInts.srv
│               ├── package.xml
│               └── CMakeLists.txt
│
├── raspberry_pi/                  # Code running on Raspberry Pi
│   └── ros_ws/
│       └── src/
│           └── motortest/         # ROS package: actuators + sensors
│               ├── scripts/
│               │   ├── motortest.py    # Drive motor controller
│               │   ├── capahareket.py  # Hoe mechanism controller
│               │   └── mesafetest.py   # MZ80 optical sensor counter
│               ├── launch/
│               │   └── aracbasla.launch  # Start all RPi-side nodes
│               ├── package.xml
│               └── CMakeLists.txt
│
├── models/
│   └── README.md                  # Instructions to obtain model weights
│
└── docs/
    └── architecture.md            # System architecture overview
```

---

## Installation

### Prerequisites

- Ubuntu 20.04 with [ROS1 Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- Python 3.8+
- `catkin_make` or `catkin build`

### 1. Clone the repository

```bash
git clone https://github.com/YOUR_USERNAME/tarimsal_insansiz_kara_araci.git
cd tarimsal_insansiz_kara_araci
```

### 2. PC Station Setup

```bash
cd pc_station/ros_ws
pip install -r ../../requirements_pc.txt

# Build the ROS workspace
catkin_make
source devel/setup.bash
```

### 3. Raspberry Pi Setup

```bash
cd raspberry_pi/ros_ws
pip install -r ../../requirements_rpi.txt

catkin_make
source devel/setup.bash
```

### 4. Model Weights

See [`models/README.md`](models/README.md) for instructions on downloading or
training the YOLOv8 crop-detection model.

---

## Environment Variables

| Variable          | Default                              | Description                          |
|-------------------|--------------------------------------|--------------------------------------|
| `YOLO_MODEL_PATH` | `pc_station/ros_ws/src/otonom/models/best.pt` | Path to the YOLO `.pt` weight file |
| `ROS_MASTER_URI`  | `http://localhost:11311`             | URI of the ROS master                |
| `ROS_IP`          | *(set per machine)*                  | Local IP for multi-machine ROS setup |

For multi-machine operation, set on the PC:

```bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=<PC_IP>
```

And on the Raspberry Pi:

```bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=<RPI_IP>
```

---

## Usage

### Start ROS Master (on PC)

```bash
roscore
```

### Launch PC-side nodes (vision + sensor bridges)

```bash
cd pc_station/ros_ws
source devel/setup.bash
roslaunch otonom veribasla.launch
```

### Launch Raspberry Pi nodes (motors + hoe + sensors)

```bash
cd raspberry_pi/ros_ws
source devel/setup.bash
roslaunch motortest aracbasla.launch
```

### Overriding the YOLO model path

```bash
rosrun otonom otonom.py _model_path:=/path/to/your/best.pt
```

---

## ROS Topic Graph

```
[/usb_cam/image_raw] --> otonom.py --> /left_angle
                                   --> /detected_plant
                                   --> /object_positions
                                   --> /object_positions_y

/left_angle   --> motortest.py (RPi)  --> drives motors
              --> capahareket.py (RPi) --> drives hoe

/sensor_counter_1/2 --> motortest.py
                    --> capahareket.py

/nerdeyim --> motortest.py
```

---

## Testing

There are no automated unit tests currently included. To verify the system:

1. Start `roscore`.
2. Publish a test image to `/usb_cam/image_raw`:
   ```bash
   rosrun image_publisher image_publisher _filename:=test_image.jpg
   ```
3. Subscribe to `/left_angle` and `/detected_plant` to confirm detection output:
   ```bash
   rostopic echo /left_angle
   rostopic echo /detected_plant
   ```

---

## Roadmap

- [ ] Replace global state in `capahareket.py` with a class-based design
- [ ] Add ROS parameter YAML file for all tunable constants (PWM, thresholds)
- [ ] Integrate row-end detection using encoder counters into a unified state machine
- [ ] Add unit tests for angle computation geometry
- [ ] Add CI workflow (GitHub Actions) for lint and import checks

---

## Known Limitations

- `capahareket.py` uses module-level global state; concurrent callback access is not thread-safe.
- `motortest.py` uses `time.sleep()` inside ROS callbacks, which blocks the callback thread.
- Row-transition timing parameters (sleep durations, encoder thresholds) are empirically tuned for one specific field; re-calibration is required for different plots.
- The `expo()` timer callback contains hardcoded sleep durations; these must be adjusted per vehicle.
- No graceful failure mode if the camera feed is interrupted during operation.

---

## License

License not specified. All rights reserved by the project authors.

---









## Author

Developed as part of a university engineering project (agricultural robotics).  
For questions, open a GitHub Issue.
