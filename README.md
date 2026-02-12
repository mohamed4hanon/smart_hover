# Smart Hoverboard ROS2 Driver (Jazzy) 🚀

This repository contains a ROS2 Jazzy driver for a Hoverboard using a **Python-based Node** that communicates via an **FTDI (Serial)** adapter. 
- It converts standard ROS2 `geometry_msgs/Twist` messages into serial packets compatible with custom hoverboard firmwares.


## 🛠 Features
- **ROS2 Jazzy** native support.
- **Serial Communication** via `pyserial`.
- **Safety Timeout**: Automatically stops the motors if the connection or command stream is lost.
- **RMW Fix**: Pre-configured to avoid Shared Memory (SHM) issues on modern ROS2 distributions.

## 🏗 Directory Structure
```text
smart_hover/
├── package.xml
├── setup.py
├── smart_hover/
│   ├── __init__.py
│   └── hover_node.py    <-- Main Driver Node
└── README.md
```

## Installation 🚀
1. Prerequisites
Ensure you have ROS2 Jazzy installed and the pyserial library:
```bash
sudo apt update
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
pip install pyserial
```
يُرجى استخدام الرمز البرمجي بحذر.

2. Build the Package
Clone this repo into your workspace src folder, then build:
```bash
cd ~/ros2_ws
colcon build --packages-select smart_hover
source install/setup.bash
```
يُرجى استخدام الرمز البرمجي بحذر.

## 🚦 How to Run
1. Set Permissions
Give read/write access to your FTDI adapter:
```bash
sudo chmod 666 /dev/ttyUSB0
```
يُرجى استخدام الرمز البرمجي بحذر.

2. Start the Driver
```bash
ros2 run smart_hover hover_node
```
يُرجى استخدام الرمز البرمجي بحذر.

3. Control via Keyboard
In a new terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=20.0
```
يُرجى استخدام الرمز البرمجي بحذر.

## 🔌 Hardware Wiring
```text
FTDI Pin	Hoverboard Pin
TX	RX
RX	TX
GND	GND
📝 License
This project is licensed under the MIT License.

---
```


## 🔢 Control Logic & Mathematics

The driver converts ROS2 standard velocity units (SI) into raw integers that the Hoverboard firmware can process.

### 1. Velocity Mapping
The node subscribes to the `/cmd_vel` topic, which provides:
- **Linear Velocity ($v$):** Measured in meters per second ($m/s$).
- **Angular Velocity ($\omega$):** Measured in radians per second ($rad/s$).

We map these values to the Hoverboard's internal range (typically $-1000$ to $1000$) using scaling factors:

$$Speed_{raw} = v \times 300$$
$$Steer_{raw} = \omega \times 200$$

### 2. Serial Packet Structure
The data is packed into a binary structure to ensure high-speed transmission and integrity. Each packet follows this format:

| Byte Offset | Field | Type | Description |
|-------------|-------|------|-------------|
| 0-1 | START_FRAME | `uint16` | Always `0xABCD` |
| 2-3 | Steer | `int16` | Differential speed ($rad/s$ scaled) |
| 4-5 | Speed | `int16` | Forward/Backward speed ($m/s$ scaled) |
| 6-7 | Checksum | `uint16` | XOR sum of all previous fields |

### 3. Checksum Calculation
To prevent erratic behavior due to serial noise, a checksum is calculated for every packet:

$$Checksum = START\_FRAME \oplus Steer \oplus Speed$$

The firmware on the Hoverboard side will discard any packet where the calculated checksum doesn't match the received one.

## About me :
- Mohamed Hanon
- ICE eng.
- Iraq



