# Building IHMC ROS 2 Messages

The messages are in [ihmc-interfaces/src/main/messages/ihmc_interfaces](../../ihmc-interfaces/src/main/messages/ihmc_interfaces)

Run the `buildImage.sh` to build the image with the messages.

Additional steps:

```
~ $ export ROS_DOMAIN_ID=<your_domain_id>

$ ros2 topic list
$ ros2 topic hz /ihmc/nadia/humanoid_control/output/robot_configuration_data
```

To build the messages manually:

```
~ $ cd colcon_ws
colcon_ws $ colcon build
colcon_ws $ . install/setup.bash
```
