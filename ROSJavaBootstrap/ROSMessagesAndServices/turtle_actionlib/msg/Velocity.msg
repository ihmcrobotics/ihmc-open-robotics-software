# Copied from turtlesim https://github.com/ros/ros_tutorials/blob/f7da7779e82dcc3977b2c220a843cd86dd269832/turtlesim/msg/Velocity.msg. We had to copy this into this package since it has been replaced with geometry_msgs/Twist there and comforming to Twist requires to change code, which I doubt worth time it takes. So if you think it is, please go ahead make a patch.

float32 linear
float32 angular
