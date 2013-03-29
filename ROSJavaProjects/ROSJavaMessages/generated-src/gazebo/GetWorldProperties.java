package gazebo;

public interface GetWorldProperties extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetWorldProperties";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\n---\nfloat64 sim_time                     # current sim time\nstring[] model_names                 # list of models in the world\nbool rendering_enabled               # if X is used\nbool success                         # return true if get successful\nstring status_message                # comments if available\n";
}
