package gazebo_msgs;

public interface GetModelProperties extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/GetModelProperties";
  static final java.lang.String _DEFINITION = "string model_name                    # name of Gazebo Model\n---\nstring parent_model_name             # parent model\nstring canonical_body_name           # name of canonical body, body names are prefixed by model name, e.g. pr2::base_link\nstring[] body_names                  # list of bodies, body names are prefixed by model name, e.g. pr2::base_link\nstring[] geom_names                  # list of geoms\nstring[] joint_names                 # list of joints attached to the model\nstring[] child_model_names           # list of child models\nbool is_static                       # returns true if model is static\nbool success                         # return true if get successful\nstring status_message                # comments if available\n";
}
