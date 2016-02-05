package arm_navigation_msgs;

public interface GetJointTrajectoryValidityResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "arm_navigation_msgs/GetJointTrajectoryValidityResponse";
  static final java.lang.String _DEFINITION = "\n# Integer error code corresponding to the first check that was violated\n# The message contains both the returned error code value and a set \n# of possible error codes\narm_navigation_msgs/ArmNavigationErrorCodes error_code\n\n# A vector of error flags for all points in the trajectory\n# Each error flag indicates which checks failed for the corresponding\n# point in the trajectory\narm_navigation_msgs/ArmNavigationErrorCodes[] trajectory_error_codes\n\narm_navigation_msgs/ContactInformation[] contacts";
  arm_navigation_msgs.ArmNavigationErrorCodes getErrorCode();
  void setErrorCode(arm_navigation_msgs.ArmNavigationErrorCodes value);
  java.util.List<arm_navigation_msgs.ArmNavigationErrorCodes> getTrajectoryErrorCodes();
  void setTrajectoryErrorCodes(java.util.List<arm_navigation_msgs.ArmNavigationErrorCodes> value);
  java.util.List<arm_navigation_msgs.ContactInformation> getContacts();
  void setContacts(java.util.List<arm_navigation_msgs.ContactInformation> value);
}
