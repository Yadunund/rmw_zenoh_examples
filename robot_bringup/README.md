# robot_bringup

[!](media/robot.png)

## Bringup

Let's assume we have a mobile manipulator.
The launch the various nodes,

First ensure the Zenoh router on the robot is running.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Then bringup the various nodes.

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch dummy_robot_bringup dummy_robot_bringup_launch.xml
```

To see the topics published by the robot,

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic list -v
```

The output should look like this (standard topics ignored)
```bash
/joint_states [sensor_msgs/msg/JointState] 1 publisher
/map [nav_msgs/msg/OccupancyGrid] 1 publisher
/robot_description [std_msgs/msg/String] 1 publisher
/scan [sensor_msgs/msg/LaserScan] 1 publisher
/tf [tf2_msgs/msg/TFMessage] 1 publisher
/tf_static [tf2_msgs/msg/TFMessage] 1 publisher
```

To visualize the robot,
```bash
source /opt/ros/jazzy/setup.bash
cd robot_bringup
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
rviz2 -d robot.rviz
```

## Tuning Zenoh configs

The following sections highlight how Zenoh configs for different robots and a cloud system can be tuned to achieve better performance, security, and scalability.

The configs can be found in [zenoh_configs](./zenoh_configs).
- [robot_router_config.json5](./zenoh_configs/obot_session_config.json5) : Config for the Zenoh sessions (ROS 2 nodes) running on the robot.
- [robot_session_config.json5](./zenoh_configs/obot_router_config.json5) : Config for the Zenoh router running on the robot.
- [cloud_router_config.json5](./zenoh_configs/obot_router_config.json5) : Config for the Zenoh router running on the cloud.
- [cloud_router_config.json5](./zenoh_configs/obot_router_config.json5) : Config for the Zenoh router running on the cloud.

### Shared memory

Shared memory should be enabled on both the router and session configs for the robot.

```json5
    shared_memory: {
      enabled: true,
      mode: "lazy",
    },
```

The default size of SHM is 16MB.
This can be adjusted by setting the `ZENOH_SHM_ALLOC_SIZE` envar to multiple of 4 that represent the number of bytes of the shared memeory segment.

Override the Zenoh router and session configs with the custom config files.

```bash
source /opt/ros/jazzy/setup.bash
cd robot_bringup
export ZENOH_ROUTER_CONFIG_UTI=zenoh_configs/robot_router_config.json5
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Then bringup the various nodes.

```bash
source /opt/ros/jazzy/setup.bash
cd robot_bringup
export ZENOH_SESSION_CONFIG_UTI=zenoh_configs/robot_session_config.json5
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch dummy_robot_bringup dummy_robot_bringup_launch.xml
```

### Congestion Control & Priority

System load is largest at startup and there is a high probability for important messages to be dropped. Eg. PointCloud, OccupancyGrid.
In `rmw_zenoh`, `KEEP_ALL` and `RELIABLE` QoS settings will force publisher to use reliable channels and always block packages.
But publishers will consume more resources.
Solution: We can edit Zenoh config to control dropping & priority policy per topic.
`blockfirst` makes congestion control more robust and fair.

The following config is present in [robot_router_config.json5](./zenoh_configs/robot_session_config.json5)
```json5
  qos: {
    publication: [
      {
        key_exprs: ["*/map/*/*"],
        config: {
          congestion_control: "blockfirst",
          priority: "data_high",
          express: true,
          reliability: "reliable",
          allowed_destination: "remote",
        },
      },
    ],
  },
```
The robot nodes can be started in the same manner as above.

### Cloud connectivity

For the purpose of this demonstration, the cloud instance will be emulated bu running a Zenoh router that listens for connections on a different local port (`17447`) on the same host. Cloud sessions will connect to this port.
The cloud router is also configured to connect to the robot's router.

With the robot nodes running, start a second cloud router
```bash
source /opt/ros/jazzy/setup.bash
cd robot_bringup
export ZENOH_ROUTER_CONFIG_UTI=zenoh_configs/cloud_router_config.json5
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Then introspect the system,

```bash
source /opt/ros/jazzy/setup.bash
cd robot_bringup
export ZENOH_SESSION_CONFIG_UTI=zenoh_configs/rcloud_session_config.json5
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 topic list -v
```

### Downsampling