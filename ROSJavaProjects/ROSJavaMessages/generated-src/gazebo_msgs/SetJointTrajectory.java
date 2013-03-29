package gazebo_msgs;

public interface SetJointTrajectory extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SetJointTrajectory";
  static final java.lang.String _DEFINITION = "string model_name\ntrajectory_msgs/JointTrajectory joint_trajectory\ngeometry_msgs/Pose model_pose\nbool set_model_pose\nbool disable_physics_updates # defaults to false\n---\nbool success                # return true if set wrench successful\nstring status_message       # comments if available\n";
}
