# Compiling the ros2 message definitions
This directory can be treated as a colcon workspace. Use
```
$ ./compile_interfaces.bash
```
to compile the IHMC ROS2 interfaces into the current colcon workspace (this directory).

After compiling the IHMC ROS2 interfaces, run
```
$ source install/setup.bash
```

Ensure you've set `ROS_DOMAIN_ID` equal to `RTPSDomainID` in `~/.ihmc/IHMCNetworkParameters.ini`. If
you have not created or edited this file, run `NetworkParametersCreator` and set `RTPSDomainID` and
`RTPSSubnet`.
```
$ export ROS_DOMAIN_ID=32 # Change to the domain ID you've set in IHMCNetworkParameters.ini
```