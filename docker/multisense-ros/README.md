# MultiSense ROS Docker Image

This images expects that you clone MultiSense ROS to a catkin ws at `/home/<user>/dev/multisense_ws/src`.

```
mkdir -p ~/dev/multisense_ws/src
cd ~/dev/multisense_ws/src
git clone --recurse-submodules https://github.com/carnegierobotics/multisense_ros.git
```

Then, run the `startMultisense.sh` script.

The container and build will be resused. To reset the container, run `# docker rm mapsense` and run the script again.

Reference the Confluence page for more:
https://confluence.ihmc.us/display/PER/MultiSense+SL