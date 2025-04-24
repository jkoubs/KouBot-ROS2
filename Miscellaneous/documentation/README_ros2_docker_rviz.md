# ROS 2 + Docker + RViz Setup with Raspberry Pi

This documentation explains how to set up and use a Raspberry Pi 4B+ to run ROS 2 inside Docker, with a shared workspace volume (`ros2_ws`), VNC for terminal access, and RViz running remotely from a laptop (also in Docker).

---

## Overview

- **Raspberry Pi 4B+**: Runs ROS 2 nodes inside a Docker container
- **Laptop**: Runs RViz in Docker to visualize data from the Pi
- **Networking**: ROS 2 discovery over LAN using DDS multicast
- **Workspace**: `ros2_ws` lives on the Pi and is volume-mounted into the container

---

## Raspberry Pi Setup

### Folder Structure

```
/home/ubuntu/KouBot-ROS2/
├── ros2_ws/
├── docker-compose.yml
├── supervisord.conf
└── vnc_passwd/
```

> `ros2_ws/src` is shared with the container, so you can build, edit, and test code natively from the host Pi.

---

## Laptop Setup (RViz)

Build a minimal Docker image with RViz2 using `ros:galactic`, and use host networking for discovery. Mount `/tmp/.X11-unix` for GUI display.

---

## ROS 2 Networking

- Use `network_mode: host` on both Pi and Laptop containers
- Set `ROS_DOMAIN_ID=10` and `ROS_LOCALHOST_ONLY=0` on both
- Enables Fast DDS discovery over LAN

---

## Testing

From the laptop (inside the RViz container):

```bash
ros2 topic list
```

You should see:

```
/scan
/tf
/robot_description
```

Then run `rviz2` and visualize topics like `/scan`.

---

## Summary

- Keeping `ros2_ws` on the Pi is efficient and flexible
- Mounting it into the Docker container enables smooth development
- No need to break into multiple services unless scaling
- VNC provides GUI access (e.g., for Terminator)
- RViz runs on the laptop for optimal performance

---


