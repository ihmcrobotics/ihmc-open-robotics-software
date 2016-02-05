package arm_navigation_msgs;

public interface MultiDOFJointTrajectory extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/MultiDOFJointTrajectory";
  static final java.lang.String _DEFINITION = "#A representation of a multi-dof joint trajectory\nduration stamp\nstring[] joint_names\nstring[] frame_ids\nstring[] child_frame_ids\nMultiDOFJointTrajectoryPoint[] points\n";
  org.ros.message.Duration getStamp();
  void setStamp(org.ros.message.Duration value);
  java.util.List<java.lang.String> getJointNames();
  void setJointNames(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getFrameIds();
  void setFrameIds(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getChildFrameIds();
  void setChildFrameIds(java.util.List<java.lang.String> value);
  java.util.List<arm_navigation_msgs.MultiDOFJointTrajectoryPoint> getPoints();
  void setPoints(java.util.List<arm_navigation_msgs.MultiDOFJointTrajectoryPoint> value);
}
