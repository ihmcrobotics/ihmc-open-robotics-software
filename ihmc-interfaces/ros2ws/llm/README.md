# Using the LLM in the Behavior System Examples
First follow the instructions related to the behavior system examples in the parent directory.
For using the LLm you will also need
- rclpy
- opencv-python
- together

You can install them with pip:
```
$ pip install rclpy opencv-python together
```

After that you will need to register YOUR_API_KEY for using Together.
You can find your API key at https://api.together.xyz/settings/api-keys.

You can either register you key in the python file:
```
os.environ["TOGETHER_API_KEY"] = "YOUR_API_KEY"
```

or store it as an ENV variable:
```
$ echo 'export TOGETHER_API_KEY="YOUR_API_KEY"' >> ~/.bashrc
$ source ~/.bashrc
```

If you do so be sure to comment out or delete this line from the python file:
```
#os.environ["TOGETHER_API_KEY"] = "YOUR_API_KEY"
```

## Notes. Errors while running examples
1. /usr/bin/python3.10 behaviors_example.py
Traceback (most recent call last):
  File "/home/sravani/nadia/repository-group/ihmc-open-robotics-software/ihmc-interfaces/ros2ws/behaviors_example.py", line 10, in <module>
    from behavior_msgs.msg import AI2RCommandMessage
ModuleNotFoundError: No module named 'behavior_msgs'

Solution: Run 
    source ~/nadia/repository-group/ihmc-open-robotics-software/ihmc-interfaces/ros2ws/install/setup.bash

Error: No output

Solution: export ROS_DOMAIN_ID=32

So Order is:
    1. source ~/nadia/repository-group/ihmc-open-robotics-software/ihmc-interfaces/ros2ws/install/setup.bash
    2. export ROS_DOMAIN_ID=32
    3. /usr/bin/python3.10 behaviors_example.py

