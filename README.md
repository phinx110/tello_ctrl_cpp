# Drone controller

The brain of the Tello drone.

#### client
```
srv_tello_cmd = create_client<tello_msgs::srv::TelloAction>("/solo/tello_action");
```
srv_tello_cmd is used to send commands to the tello driver.

#### subscribers

```
sub_key_input       = create_subscription<std_msgs::msg::Char>            ("/raw_keyboard", ..., ...);
sub_tello_response  = create_subscription<tello_msgs::msg::TelloResponse> ("/solo/tello_response", ..., ...);
sub_demo_position   = create_subscription<geometry_msgs::msg::Point>      ("/solo/demo_position", ..., ...);
```
sub_key_input is used as input, see state machine below. sub_tello_response is used to receive feedback from tello driver. sub_demo_position is used to updated the point where the drone needs to go to.

#### Publishers
```
pub_tello_twist = create_publisher<geometry_msgs::msg::Twist>("/solo/cmd_vel", ...);
```
pub_tello_twist publisches twist comands to the tello driver to move the drone.

#### State machine

Image below is the state machine. Press 'q' to either liftoff, land or abort search, and 'w' to search for the Aruco marker.

![alt text](https://raw.githubusercontent.com/phinx110/tello_ctrl_cpp/master/state_machine.png)

#### Modifying position

Use the following command in the terminal to move the drone to a custom position:
```
ros2 topic pub /solo/demo_position geometry_msgs/Point "{x: 1.2, y: 0.5, z: 2.5}"
```
