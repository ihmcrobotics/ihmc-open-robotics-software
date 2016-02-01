package arm_navigation_msgs;

public interface LinkPadding extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/LinkPadding";
  static final java.lang.String _DEFINITION = "#name for the link\nstring link_name\n\n# padding to apply to the link\nfloat64 padding\n";
  java.lang.String getLinkName();
  void setLinkName(java.lang.String value);
  double getPadding();
  void setPadding(double value);
}
