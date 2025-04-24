### 0. Build Arduino Code

Open Platformio by clicking on the icon on the right side of VSCode. Then open the projects located at: **/home/ubuntu/Documents/PlatformIO/Projects/Mecanum wheels**.

```bash
platformio remote agent start
```

REMOTE UPLOAD


### 1. Run Koubot VNC Container [RPi]

```bash
cd /home/ubuntu/KouBot-ROS2/docker
docker-compose -f docker-compose-koubot-vnc-v4.yml down
docker-compose -f docker-compose-koubot-vnc-v4.yml up
```

### 2. Launch VNC Viewer [Local]

```bash
vncviewer 192.168.1.221:5901
```

### 3. Launch ROS 2 nodes [Terminator-TigerVNC]

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

* **Terminal 4: launch Scan matcher Odom**
```bash
cd /ros2_ws/
colcon build --packages-select ros2_laser_scan_matcher
source install/setup.bash
ros2 launch ros2_laser_scan_matcher start_matcher.launch.py
```

* **Terminal 5: Mapping with SLAM Toolbox**
```bash
cd /ros2_ws/
colcon build --packages-select mecanum_interface
source install/setup.bash
ros2 launch mecanum_interface slam_toolbox_lidar_only.launch.py
```

* **Terminal 6: Save the Map**
```bash
cd /ros2_ws/src/mecanum_interface/maps
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 4. Run RViz VNC Container [Local]

```bash
cd /home/jason/Projects/ros2_galactic/docker
docker-compose -f docker-compose-vnc-rviz.yml up
```

### 5. Launch RViz Locally [Terminator]

```bash
rviz2
```

Now you can add the Displays in ROS 2 such as:

* TF
* RobotModel
* LaserScan

**Note:** Don't forget to set the **`Fixed Frame`** to **`base_link`**.

