package object_manipulation_msgs;

public interface Grasp extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/Grasp";
  static final java.lang.String _DEFINITION = "\n# The internal posture of the hand for the pre-grasp\n# only positions are used\nsensor_msgs/JointState pre_grasp_posture\n\n# The internal posture of the hand for the grasp\n# positions and efforts are used\nsensor_msgs/JointState grasp_posture\n\n# The position of the end-effector for the grasp relative to a reference frame \n# (that is always specified elsewhere, not in this message)\ngeometry_msgs/Pose grasp_pose\n\n# The estimated probability of success for this grasp\nfloat64 success_probability\n\n# Debug flag to indicate that this grasp would be the best in its cluster\nbool cluster_rep\n\n# how far the pre-grasp should ideally be away from the grasp\nfloat32 desired_approach_distance\n\n# how much distance between pre-grasp and grasp must actually be feasible \n# for the grasp not to be rejected\nfloat32 min_approach_distance\n\n# an optional list of obstacles that we have semantic information about\n# and that we expect might move in the course of executing this grasp\n# the grasp planner is expected to make sure they move in an OK way; during\n# execution, grasp executors will not check for collisions against these objects\nGraspableObject[] moved_obstacles\n";
  sensor_msgs.JointState getPreGraspPosture();
  void setPreGraspPosture(sensor_msgs.JointState value);
  sensor_msgs.JointState getGraspPosture();
  void setGraspPosture(sensor_msgs.JointState value);
  geometry_msgs.Pose getGraspPose();
  void setGraspPose(geometry_msgs.Pose value);
  double getSuccessProbability();
  void setSuccessProbability(double value);
  boolean getClusterRep();
  void setClusterRep(boolean value);
  float getDesiredApproachDistance();
  void setDesiredApproachDistance(float value);
  float getMinApproachDistance();
  void setMinApproachDistance(float value);
  java.util.List<object_manipulation_msgs.GraspableObject> getMovedObstacles();
  void setMovedObstacles(java.util.List<object_manipulation_msgs.GraspableObject> value);
}
