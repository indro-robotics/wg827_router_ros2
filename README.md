# wg827_router_ros2

This package allows to obtain modem information and publish it to `/modem/modem_info` ROS2 topic.

# install

Please install dependency by running the following command:
```
pip install spur
```

# Build

To build the package please run following command from the workspace:
```
colcon build
source install/setup.bash
```

# launch 

To launch the node please run the following command.
```
ros2 launch modem_info modem_info.launch.py
```
