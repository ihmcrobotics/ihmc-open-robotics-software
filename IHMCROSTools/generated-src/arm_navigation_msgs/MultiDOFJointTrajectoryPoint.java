package arm_navigation_msgs;

public interface MultiDOFJointTrajectoryPoint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/MultiDOFJointTrajectoryPoint";
  static final java.lang.String _DEFINITION = "geometry_msgs/Pose[] poses\nduration time_from_start";
  java.util.List<geometry_msgs.Pose> getPoses();
  void setPoses(java.util.List<geometry_msgs.Pose> value);
  org.ros.message.Duration getTimeFromStart();
  void setTimeFromStart(org.ros.message.Duration value);
}
