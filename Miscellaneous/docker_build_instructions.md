# Modify `Dockerfile.koubot-vnc` located at:

```bash
cd /home/jason/Projects/KouBot-ROS2/docker
git checkout hardware_integration
```

# Register QEMU emulators inside your Docker environment so you can build and run containers for other architectures (like arm64) on an amd64 system (e.g. your laptop) [LOCAL]

```bash
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
```

# Build and push only the ARM64 version (for Raspberry Pi and other ARM devices), not multi-platform [LOCAL]

```bash
cd /home/jason/Projects/KouBot-ROS2/docker
git checkout hardware_integration
docker buildx build --platform linux/arm64   -t jkoubi/koubot_vnc:v4 --no-cache --push -f Dockerfile.koubot-vnc ..
```

# (OPTIONAL) Multi-platform build (for both PC + Pi) [LOCAL]

```bash
docker buildx build --platform linux/amd64,linux/arm64 -t jkoubi/koubot_vnc:v4 --no-cache --push -f Dockerfile.koubot-vnc ..
```

# Pull Docker Image from Docker Hub into Pi [Pi]

```bash
docker pull jkoubi/koubot_vnc:v4
```

# Run Container [Pi]

**Note: Before running container need to update the docker image name in the docker-compose.yml**


```bash
cd /home/ubuntu/KouBot-ROS2/docker
docker-compose -f docker-compose-koubot-vnc.yml down
docker-compose -f docker-compose-koubot-vnc.yml up
```