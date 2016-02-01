package gazebo_msgs;

public interface GetModelState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/GetModelState";
  static final java.lang.String _DEFINITION = "string model_name                    # name of Gazebo Model\nstring relative_entity_name          # return pose and twist relative to this entity\n                                     # an entity can be a model, body, or geom\n                                     # be sure to use gazebo scoped naming notation (e.g. [model_name::body_name])\n                                     # leave empty or \"world\" will use inertial world frame\n---\ngeometry_msgs/Pose pose              # pose of model in relative entity frame\ngeometry_msgs/Twist twist            # twist of model in relative entity frame\nbool success                         # return true if get successful\nstring status_message                # comments if available\n";
}
