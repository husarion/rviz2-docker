# rviz2-docker

Building a Docker image with RViz2 and Nav2 plugin.

Available for ROS distros:
- ROS 2 galactic
- ROS 2 humble

Create the following `compose.yaml` file:
```yaml
version: "2.3"

services:
  rviz:
    image: husarion/rviz2
    network_mode: host
    ipc: host
    runtime: nvidia
    environment:
      - ROS_DOMAIN_ID=2
      - DISPLAY
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./rosbot_pro.rviz:/root/.rviz2/default.rviz
```

And execute

```bash
xhost local:root
docker compose up
```