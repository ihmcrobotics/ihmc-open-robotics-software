package atlas_msgs;

public interface AtlasBehaviorStepData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorStepData";
  static final java.lang.String _DEFINITION = "# multi_step walking trajectory parameters\nuint32 step_index              # Step index, matlab style, starting from 1,\n                               # monotonically increasing during walking\n                               #  resets to 1 if robot leaves walk behaviors\nuint32 foot_index              # Foot_index can be LEFT_FOOT or RIGHT_FOOT\nfloat64 duration               # Step duration, when in doubt, 0.63s is a\n                               # good guess.\ngeometry_msgs/Pose pose        # Foot pose in Atlas world frame\nfloat64 swing_height           # Step apex swing height measured form the\n                               # midpoint between the feet.\n";
  int getStepIndex();
  void setStepIndex(int value);
  int getFootIndex();
  void setFootIndex(int value);
  double getDuration();
  void setDuration(double value);
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
  double getSwingHeight();
  void setSwingHeight(double value);
}
