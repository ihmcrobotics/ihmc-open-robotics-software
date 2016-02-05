package arm_navigation_msgs;

public interface GetStateValidityResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetStateValidityResponse";
  static final java.lang.String _DEFINITION = "\n# Integer error code corresponding to the first check that was violated\n# The message contains both the returned error code value and a set \n# of possible error codes\narm_navigation_msgs/ArmNavigationErrorCodes error_code\n\n# Contact information\narm_navigation_msgs/ContactInformation[] contacts";
  arm_navigation_msgs.ArmNavigationErrorCodes getErrorCode();
  void setErrorCode(arm_navigation_msgs.ArmNavigationErrorCodes value);
  java.util.List<arm_navigation_msgs.ContactInformation> getContacts();
  void setContacts(java.util.List<arm_navigation_msgs.ContactInformation> value);
}
