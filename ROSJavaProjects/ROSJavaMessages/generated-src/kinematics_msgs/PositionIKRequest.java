package kinematics_msgs;

public interface PositionIKRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinematics_msgs/PositionIKRequest";
  static final java.lang.String _DEFINITION = "# A Position IK request message\n# The name of the link for which we are computing IK\nstring ik_link_name\n\n# The (stamped) pose of the link\ngeometry_msgs/PoseStamped pose_stamped\n\n# A RobotState consisting of hint/seed positions for the IK computation. \n# These may be used to seed the IK search. \n# The seed state MUST contain state for all joints to be used by the IK solver\n# to compute IK. The list of joints that the IK solver deals with can be found using\n# the kinematics_msgs/GetKinematicSolverInfo\narm_navigation_msgs/RobotState ik_seed_state\n\n# Additional state information can be provided here to specify the starting positions \n# of other joints/links on the robot.\narm_navigation_msgs/RobotState robot_state\n";
  java.lang.String getIkLinkName();
  void setIkLinkName(java.lang.String value);
  geometry_msgs.PoseStamped getPoseStamped();
  void setPoseStamped(geometry_msgs.PoseStamped value);
  arm_navigation_msgs.RobotState getIkSeedState();
  void setIkSeedState(arm_navigation_msgs.RobotState value);
  arm_navigation_msgs.RobotState getRobotState();
  void setRobotState(arm_navigation_msgs.RobotState value);
}
