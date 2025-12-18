# Using the Turtlebot3 for Intelligent Automation

## How to use

1. Your computer must be running the docker container. Considering that you are using ROS2 Humble:

```bash
cd docker/humble
chmod +x container.sh
./container.sh start 
./container.sh enter
```

2. Ensures that the Turtlebot is up running the bringup package: 

```bash
ssh ubuntu@turtlebot-vt1050.local # the password is 1234
cd turtlebot3_ws/src/turtlebot3
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_bringup robot.launch.py
```

- Obs: If you are not seeing the ros topics, ensures that you are in the same Domain ID. By default, the turtlebot3 domain id is 30.

```bash
export ROS_DOMAIN_ID=30
```