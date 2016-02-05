package gazebo_msgs;

public interface SpawnModel extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/SpawnModel";
  static final java.lang.String _DEFINITION = "string model_name                 # name of the model to be spawn\nstring model_xml                  # this should be an urdf or gazebo xml\nstring robot_namespace            # spawn robot and all ROS interfaces under this namespace\ngeometry_msgs/Pose initial_pose   # only applied to canonical body\nstring reference_frame            # initial_pose is defined relative to the frame of this model/body\n                                  # if left empty or \"world\", then gazebo world frame is used\n                                  # if non-existent model/body is specified, an error is returned\n                                  #   and the model is not spawned\n---\nbool success                      # return true if spawn successful\nstring status_message             # comments if available\n";
}
