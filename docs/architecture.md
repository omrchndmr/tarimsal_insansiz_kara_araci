# System Architecture

## Overview

The system uses two separate ROS nodes distributed across two machines:

```
┌──────────────────────────────────────────────────────────────────┐
│  PC Station (Ubuntu 20.04 + ROS Noetic)                          │
│                                                                  │
│  USB Camera ──► otonom.py ──► /left_angle                        │
│                           ──► /detected_plant                    │
│                           ──► /object_positions                  │
│                           ──► /object_positions_y                │
│                                                                  │
│  Arduino (BNO055) ──► aci.py ──► /gyro_data                     │
│  Arduino (sonar)  ──► listener.py ──► /sensor_0_distance        │
│  Arduino (flex)   ──► flexveri.py ──► /analog_sensor_value      │
│  Arduino (HC-SR04)──► mesafeveri.py ──► /arduino_data           │
└──────────────────────────────────────────────────────────────────┘
                          │  Wi-Fi / Ethernet (ROS network)
                          ▼
┌──────────────────────────────────────────────────────────────────┐
│  Raspberry Pi (RPi OS 64-bit + ROS Noetic)                       │
│                                                                  │
│  /left_angle ──► motortest.py ──► GPIO PWM ──► Drive Motors     │
│  /detected_plant                                                 │
│  /nerdeyim  ──► motortest.py  (zone state machine)              │
│                                                                  │
│  /detected_plant ──► capahareket.py ──► GPIO PWM ──► Hoe Motor  │
│  /object_positions_y                                             │
│  /sensor_counter_1/2                                             │
│                                                                  │
│  MZ80 sensors ──► mesafetest.py ──► /sensor_counter_1/2         │
└──────────────────────────────────────────────────────────────────┘
```

## Operation Zones (nerdeyim)

| Value | Description                                       |
|-------|---------------------------------------------------|
| 0     | Traversing crop row (vision steering active)      |
| 1     | Same as 0, used for second traversal logic        |
| 2     | Row transition — hoe raised, vehicle turning      |
| 3     | Traversing second crop row                        |
| 4     | End of operation — vehicle stops                  |

## Steering Logic

The `otonom.py` node detects the two closest plants and computes the angle
between:
- A **fixed baseline** at image bottom (left_wheel_x, right_wheel_x)
- A **line from the steer origin** to the midpoint of the two plants

If the midpoint is left of `STEER_ORIGIN_X`, the angle is reflected to
indicate a left-of-center deviation.

The published `left_angle` is compared to `[88.5, 91.5]` degrees in
`motortest.py`:
- In range → straight (durDumen + forward)
- Below → steer right
- Above → steer left
