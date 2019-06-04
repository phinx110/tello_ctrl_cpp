# Drone controller

Comunicates with the DJI Tello drone through the tello_ros driver and listens to ros2_raw_keyboard_publisher for input. The source code is mostly self-explanatory.

It contains a statemashine that controlls the Tello drone movements.

Press 'q' to either liftoff, land or abort marker search, and 'w' to search for the Aruco marker.

Use the following command in the terminal to move the drone to a custom position:
```
ros2 topic pub /solo/demo_position geometry_msgs/Point "{x: -2.5, y: 0.2, z: 2.2}"
```
