# ROS node for ATI Mini45 Force Sensor

## Running the Node

For example:
```
roslaunch force_sensor_ros_interface main.launch
```

The arguments and their default values can be seen below.
- "com_port" - default: "ttyUSB0" - The port where the force sensor is connected.
- "baud_rate" - default: 19200 - The baudrate of the force sensor.

For example:
```
roslaunch force_sensor_ros_interface main.launch com_port:="ttyACM0"
```

