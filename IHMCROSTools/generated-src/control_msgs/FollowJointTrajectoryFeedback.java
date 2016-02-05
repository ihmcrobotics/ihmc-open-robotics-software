package control_msgs;

public interface FollowJointTrajectoryFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "control_msgs/FollowJointTrajectoryFeedback";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\nHeader header\nstring[] joint_names\ntrajectory_msgs/JointTrajectoryPoint desired\ntrajectory_msgs/JointTrajectoryPoint actual\ntrajectory_msgs/JointTrajectoryPoint error\n\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<java.lang.String> getJointNames();
  void setJointNames(java.util.List<java.lang.String> value);
  trajectory_msgs.JointTrajectoryPoint getDesired();
  void setDesired(trajectory_msgs.JointTrajectoryPoint value);
  trajectory_msgs.JointTrajectoryPoint getActual();
  void setActual(trajectory_msgs.JointTrajectoryPoint value);
  trajectory_msgs.JointTrajectoryPoint getError();
  void setError(trajectory_msgs.JointTrajectoryPoint value);
}
