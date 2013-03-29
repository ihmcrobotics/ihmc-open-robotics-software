package gazebo;

public interface GetModelProperties extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetModelProperties";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring model_name                    # name of Gazebo Model\n---\nstring parent_model_name             # parent model\nstring canonical_body_name           # name of canonical body\nstring[] body_names                  # list of bodies\nstring[] geom_names                  # list of geoms\nstring[] joint_names                 # list of joints attached to the model\nstring[] child_model_names           # list of child models\nbool is_static                       # returns true if model is static\nbool success                         # return true if get successful\nstring status_message                # comments if available\n";
}
