# Building IHMC ROS 2 Messages

The messages are in [ihmc-interfaces/src/main/messages/ihmc_interfaces](../../ihmc-interfaces/src/main/messages/ihmc_interfaces)

Run the `temporarilyCopyMessagerHere.sh` to build the image with the messages. Remove the copied `ihmc_interfaces` folder afterwards.

Additional steps:

```
~ $ export ROS_DOMAIN_ID=<your_domain_id>
~ $ cd colcon_ws
colcon_ws $ colcon build
colcon_ws $ . install/setup.bash

$ ros2 topic list
$ ros2 topic hz /ihmc/nadia/humanoid_control/output/robot_configuration_data
```

