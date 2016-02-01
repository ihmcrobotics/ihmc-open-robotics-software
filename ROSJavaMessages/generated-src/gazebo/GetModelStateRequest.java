package gazebo;

public interface GetModelStateRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetModelStateRequest";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring model_name                    # name of Gazebo Model\nstring relative_entity_name          # return pose and twist relative to this entity\n                                     # an entity can be a model, body, or geom\n                                     # be sure to use gazebo notation (e.g. [model_name::body_name])\n                                     # leave empty or \"world\" will use inertial world frame\n";
  java.lang.String getModelName();
  void setModelName(java.lang.String value);
  java.lang.String getRelativeEntityName();
  void setRelativeEntityName(java.lang.String value);
}
