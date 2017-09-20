package sandia_hand_msgs;

public interface SimpleGrasp extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "sandia_hand_msgs/SimpleGrasp";
  static final java.lang.String _DEFINITION = "string  name\nfloat64 closed_amount  \n# closed_amount = 0 means fully open\n# closed_amount = 1 means fully closed\n";
  java.lang.String getName();
  void setName(java.lang.String value);
  double getClosedAmount();
  void setClosedAmount(double value);
}
