# twist_stamper
ROS2 package for converting Twist messages to TwistStamped.


## Usage


- `/cmd_vel_in` - A `Twist` topic to listen on
- `/cmd_vel_out` - A `TwistStamped` topic to publish
- `frame_id` - Value to set for the output frame ID (default `''`)

For example, if you had a node such as `teleop_twist_joy` publishing a `Twist` message on the topic `/cmd_vel` and something expecting a `TwistStamped` message in the `my_frame` frame on `/my_node/cmd_vel` you would use the following command:


```ros2 run twist_stamper twist_stamper --ros-args -r cmd_vel_in:=cmd_vel -r cmd_vel_out:=my_node/cmd_vel -p frame_id:=my_frame```

If unsure, leave `frame_id` empty.