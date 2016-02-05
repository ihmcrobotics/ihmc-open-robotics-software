package turtle_actionlib;

public interface Velocity extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "turtle_actionlib/Velocity";
  static final java.lang.String _DEFINITION = "# Copied from turtlesim https://github.com/ros/ros_tutorials/blob/f7da7779e82dcc3977b2c220a843cd86dd269832/turtlesim/msg/Velocity.msg. We had to copy this into this package since it has been replaced with geometry_msgs/Twist there and comforming to Twist requires to change code, which I doubt worth time it takes. So if you think it is, please go ahead make a patch.\n\nfloat32 linear\nfloat32 angular\n";
  float getLinear();
  void setLinear(float value);
  float getAngular();
  void setAngular(float value);
}
