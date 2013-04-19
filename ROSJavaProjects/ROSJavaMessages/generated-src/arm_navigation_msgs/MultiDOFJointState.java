package arm_navigation_msgs;

public interface MultiDOFJointState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/MultiDOFJointState";
  static final java.lang.String _DEFINITION = "#A representation of a multi-dof joint state\ntime stamp\nstring[] joint_names\nstring[] frame_ids\nstring[] child_frame_ids\ngeometry_msgs/Pose[] poses\n";
  org.ros.message.Time getStamp();
  void setStamp(org.ros.message.Time value);
  java.util.List<java.lang.String> getJointNames();
  void setJointNames(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getFrameIds();
  void setFrameIds(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getChildFrameIds();
  void setChildFrameIds(java.util.List<java.lang.String> value);
  java.util.List<geometry_msgs.Pose> getPoses();
  void setPoses(java.util.List<geometry_msgs.Pose> value);
}
