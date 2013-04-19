package chomp_motion_planner;

public interface GetChompCollisionCost extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "chomp_motion_planner/GetChompCollisionCost";
  static final java.lang.String _DEFINITION = "# Array of string names for the links for which cost information is desired\nstring[] links\n\n# The state of the robot represented as an array of Kinematic states which\n# include the joint name and joint value\narm_navigation_msgs/RobotState state\n\n---\n\n# Returns an array of costs for the links (in the same order as the input)\nfloat64[] costs\n\n# Each element of this array is a num_joints x 1 array that represents the\n# joint velocity that will move the link down the gradient of the distance\n# field (away from collisions)\nJointVelocityArray[] gradient\n\n# A boolean value which indicates whether the configuration is in collision\nuint8 in_collision\n";
}
