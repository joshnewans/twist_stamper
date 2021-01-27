# twist_stamper
ROS2 package for converting between Twist and TwistStamped messages.

The package contains two nodes, `twist_stamper` and `twist_unstamper` which allow for easy conversion each way.

## Usage

### twist_stamper

The `twist_stamper` node converts `Twist` messages to `TwistStamped`. The topics and parameters are:

- `/cmd_vel_in` - A `Twist` topic to listen on
- `/cmd_vel_out` - A `TwistStamped` topic to publish
- `frame_id` - Parameter value to set for the output frame ID (default `''`)

For example, if you had a node such as `teleop_twist_joy` publishing a `Twist` message on the topic `/cmd_vel` and something expecting a `TwistStamped` message in the `my_frame` frame on `/my_node/cmd_vel` you would use the following command:


```ros2 run twist_stamper twist_stamper --ros-args -r cmd_vel_in:=cmd_vel -r cmd_vel_out:=my_node/cmd_vel -p frame_id:=my_frame```

If unsure, leave `frame_id` empty.


### twist_unstamper

The `twist_unstamper` node converts `TwistStamped` messages to `Twist`. The topics and parameters are:

- `/cmd_vel_in` - A `TwistStamped` topic to listen on
- `/cmd_vel_out` - A `Twist` topic to publish

For example, if you had a node publishing a `TwistStamped` message on the topic `/cmd_vel_stamped` and something expecting a `Twist` message on `/my_node/cmd_vel` you would use the following command:


```ros2 run twist_stamper twist_unstamper --ros-args -r cmd_vel_in:=cmd_vel_stamped -r cmd_vel_out:=my_node/cmd_vel```