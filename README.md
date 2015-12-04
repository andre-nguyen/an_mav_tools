# an_mav_tools
This package contains a set of nodes which should serve as examples on how to integrate the px4 flight stack into larger systems (or how to build a system around the px4 flight stack).

## Offboard control
The offb_control_node is a ros node to demonstrate take off by sending commands using a companion computer or some serial link to a Pixhawk. A [similar node](http://dev.px4.io/ros-mavros-offboard.html) has been contributed to the px4 development guide.

## Odometry publishing (in progress)
The posetwist2odom node subscribes to the mavros topics `local_position/pose` and `local_position/pose` to combine them into the `nav_msgs/Odometry` message. This will allow compatibility with the ros navigation stack.

## Obstacle avoid (in progress)
The obstacle avoidance node helps bridge the gap between the ros navigation stack and the px4 flight stack. Navigation is done by assuming a 2D world. Therefore a PID loop is implemented to control altitude while x and y velocities are controled by the navigation stack. Eventually altitude will be controlled onboard once support decoupled axis control is integrated into the flight stack and mavros.
