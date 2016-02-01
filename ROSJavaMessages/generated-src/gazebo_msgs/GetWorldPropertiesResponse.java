package gazebo_msgs;

public interface GetWorldPropertiesResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/GetWorldPropertiesResponse";
  static final java.lang.String _DEFINITION = "float64 sim_time                     # current sim time\nstring[] model_names                 # list of models in the world\nbool rendering_enabled               # if X is used\nbool success                         # return true if get successful\nstring status_message                # comments if available";
  double getSimTime();
  void setSimTime(double value);
  java.util.List<java.lang.String> getModelNames();
  void setModelNames(java.util.List<java.lang.String> value);
  boolean getRenderingEnabled();
  void setRenderingEnabled(boolean value);
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}
