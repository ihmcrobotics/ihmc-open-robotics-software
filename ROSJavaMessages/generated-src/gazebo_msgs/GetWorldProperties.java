package gazebo_msgs;

public interface GetWorldProperties extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/GetWorldProperties";
  static final java.lang.String _DEFINITION = "---\nfloat64 sim_time                     # current sim time\nstring[] model_names                 # list of models in the world\nbool rendering_enabled               # if X is used\nbool success                         # return true if get successful\nstring status_message                # comments if available\n";
}
