# ROS node for ATI Mini45 Force Sensor

Interfaces with the ATI Mini45 force sensor through USB (idk to what protocol) and publishes the readings to ```/force_measurements```.

## Running the Node

To run the node use:
```
roslaunch ros_ati_mini45 main.launch
```

The arguments and their default values can be seen below.
- "com_port" - default: "ttyUSB0" - The port where the force sensor is connected.
- "baud_rate" - default: 19200 - The baudrate of the force sensor.

For example:
```
roslaunch ros_ati_mini45 main.launch com_port:="/dev/ttyACM0"
```

The node also supports zeroing the force sensor, an example command can be seen below:
```
rostopic pub /zero_force_sensor ros_ati_mini45/ZeroingDuration "{ duration: 3 }"
```

Where the duration is in seconds and controls how long the sensor data is averaged for zeroing.


