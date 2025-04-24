# ROS 2 + Arduino Mecanum Wheel Control

This part enables real-time control of a 4-wheeled mecanum robot using ROS 2 on a Raspberry Pi 4B+ and an Arduino Mega 2560 over a UART (serial) connection. The ROS 2 side handles high-level velocity commands (`/cmd_vel`), while the Arduino handles low-level motor control and executes the commands on the wheels.

## How It Works

1. **ROS 2 (Raspberry Pi 4B+)** 
   - Subscribes to the `/cmd_vel` topic (of type `geometry_msgs/Twist`).
   - Extracts the linear (`vx`, `vy`) and angular (`wz`) velocity values.
   - Formats the values into a simple string: `"vx vy wz\n"` (e.g., `0.2 0.0 0.1\n`).
   - Sends the string over a serial (UART) connection to the Arduino.

2. **Arduino (Mega 2560)**  
   - Listens for incoming strings on the serial port (e.g., `"0.2 0.0 0.1"`).
   - Parses the `vx`, `vy`, and `wz` values from the string.
   - Applies mecanum drive kinematics to compute individual wheel speeds.
   - Sends PWM signals to control the direction and speed of each motor accordingly.

This allows smooth and responsive control of the robot’s motion using velocity commands from ROS 2.

## Hardware Requirements

- ROS 2-compatible computer (Raspberry Pi running ROS 2 Galactic or Humble, in my case I am using Docker which ahs a ROS 2 galactic image)
- Arduino Mega2560 or compatible board
- 4 DC motors with mecanum wheels
- 2x L298N Dual H-Bridge Motor Drivers
- USB cable for Arduino (used as a UART interface)

## Software Dependencies
```bash
# Install dependencies
pip install pyserial
```

## Launch Instructions

### 1. Compile & Upload Arduino Firmware (PlatformIO)

Make sure your PlatformIO agent is running:

```bash
platformio remote agent start
```

Then upload your firmware to the Arduino Mega 2560 using `PlatformIO Remote Upload`.


### 2. Launch ROS 2 Nodes

* **Terminal 1: Launch Robot Description**
```bash
cd /ros2_ws/
colcon build --packages-select robot_description
source install/setup.bash
ros2 launch robot_description real_robot_launch.py
```

* **Terminal 2: Start Serial Interface**
```bash
cd /ros2_ws/
colcon build --packages-select mecanum_interface
source install/setup.bash
ros2 run mecanum_interface serial_cmd_vel --ros-args -p serial_port:=/dev/ttyACM0
```

* **Terminal 3: Enable PS4 controller teleoperation**
```bash
cd /ros2_ws/
colcon build --packages-select mecanum_interface
source install/setup.bash
ros2 launch mecanum_interface ps4_teleop.launch.py
```