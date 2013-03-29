package pr2_mechanism_msgs;

public interface JointStatistics extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_mechanism_msgs/JointStatistics";
  static final java.lang.String _DEFINITION = "# This message contains the state of one joint of the pr2 robot.\n# This message is specificly designed for the pr2 robot. \n# A generic joint state message can be found in sensor_msgs::JointState\n\n# the name of the joint\nstring name\n\n# the time at which these joint statistics were measured\ntime timestamp\n\n# the position of the joint in radians\nfloat64 position\n\n# the velocity of the joint in radians per second\nfloat64 velocity\n\n# the measured joint effort \nfloat64 measured_effort\n\n# the effort that was commanded to the joint.\n# the actual applied effort might be different\n# because the safety code can limit the effort\n# a joint can apply\nfloat64 commanded_effort\n\n# a flag indicating if the joint is calibrated or not\nbool is_calibrated\n\n# a flag inidcating if the joint violated one of its position/velocity/effort limits\n# in the last publish cycle\nbool violated_limits\n\n# the total distance travelled by the joint, measured in radians.\nfloat64 odometer\n\n# the lowest position reached by the joint in the last publish cycle\nfloat64 min_position\n\n# the highest position reached by the joint in the last publish cycle\nfloat64 max_position\n\n# the maximum absolute velocity reached by the joint in the last publish cycle\nfloat64 max_abs_velocity\n\n# the maximum absolute effort applied by the joint in the last publish cycle\nfloat64 max_abs_effort\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  org.ros.message.Time getTimestamp();
  void setTimestamp(org.ros.message.Time value);
  double getPosition();
  void setPosition(double value);
  double getVelocity();
  void setVelocity(double value);
  double getMeasuredEffort();
  void setMeasuredEffort(double value);
  double getCommandedEffort();
  void setCommandedEffort(double value);
  boolean getIsCalibrated();
  void setIsCalibrated(boolean value);
  boolean getViolatedLimits();
  void setViolatedLimits(boolean value);
  double getOdometer();
  void setOdometer(double value);
  double getMinPosition();
  void setMinPosition(double value);
  double getMaxPosition();
  void setMaxPosition(double value);
  double getMaxAbsVelocity();
  void setMaxAbsVelocity(double value);
  double getMaxAbsEffort();
  void setMaxAbsEffort(double value);
}
