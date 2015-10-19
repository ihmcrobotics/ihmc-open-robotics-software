package gazebo;

public interface GetModelState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetModelState";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring model_name                    # name of Gazebo Model\nstring relative_entity_name          # return pose and twist relative to this entity\n                                     # an entity can be a model, body, or geom\n                                     # be sure to use gazebo notation (e.g. [model_name::body_name])\n                                     # leave empty or \"world\" will use inertial world frame\n---\ngeometry_msgs/Pose pose              # pose of model in relative entity frame\ngeometry_msgs/Twist twist            # twist of model in relative entity frame\nbool success                         # return true if get successful\nstring status_message                # comments if available\n";
}
