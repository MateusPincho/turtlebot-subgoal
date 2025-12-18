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


## How to create a new map with SLAM?

1. Execute the robot launch in the turtlebot SBC
2. Open a new terminal from Remote PC and launch the SLAM node. The Cartographer is used as a default SLAM method. 

```bash
 ros2 launch turtlebot3_cartographer cartographer.launch.py
```
3. Open a new terminal and run the teleoperation node from the Remote PC.

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

### How to save the generated map?
> The map is drawn based on the robot’s odometry, tf and scan information. These map data is drawn in the RViz window as the TurtleBot3 was traveling. After creating a complete map of desired area, save the map data to the local drive for the later use.

1. Execute in the Remote PC, after completing the teleop
```bash
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_ws/src/turtlebot3/map
```