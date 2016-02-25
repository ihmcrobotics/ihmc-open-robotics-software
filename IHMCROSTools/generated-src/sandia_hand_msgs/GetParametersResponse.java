package sandia_hand_msgs;

public interface GetParametersResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sandia_hand_msgs/GetParametersResponse";
  static final java.lang.String _DEFINITION = "Parameter[] parameters";
  java.util.List<sandia_hand_msgs.Parameter> getParameters();
  void setParameters(java.util.List<sandia_hand_msgs.Parameter> value);
}
