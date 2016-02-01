package atlas_msgs;

public interface AtlasBehaviorFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorFeedback";
  static final java.lang.String _DEFINITION = "# mirrors AtlasBehaviorFeedback\n#\n# Transition flags:\n#    - STATUS_TRANSITION_IN_PROGRESS\n#\n#        A transition is in progress.\n#\n#    - STATUS_TRANSITION_SUCCESS\n#\n#        Successful transition.\n#\n#    - STATUS_FAILED_TRANS_UNKNOWN_BEHAVIOR\n#\n#        Failed to transition; unknown behavior.\n#\n#    - STATUS_FAILED_TRANS_ILLEGAL_BEHAVIOR\n#\n#        Denied request for an illegal behavior transition.  This may\n#        happen if a transition to a new behavior is requested without\n#        going through a required intermediate behavior. (e.g., can\'t\n#        go from Walk straight to Manipulate.)\n#\n#    - STATUS_FAILED_TRANS_COM_POS\n#\n#        Failed to transition; the position of the COM is too far from\n#        the center of support.\n#\n#    - STATUS_FAILED_TRANS_COM_VEL\n#\n#        Failed to transition; the COM velocity too high.\n#\n#    - STATUS_FAILED_TRANS_VEL\n#\n#        Failed to transition; some joint velocities too high.\n#\n#  \\em Warnings:\n#\n#    - STATUS_WARNING_AUTO_TRANS\n#\n#        An automatic transition occurred; see behavior specific\n#        feedback for reason.\n#\n#  \\em Errors:\n#\n#    - STATUS_ERROR_FALLING\n#\n#        COM below acceptable threshold, cannot recover.\n\n# copied from AtlasBehaviorFlags\nuint32 STATUS_OK                            = 0\nuint32 STATUS_TRANSITION_IN_PROGRESS        = 1\nuint32 STATUS_TRANSITION_SUCCESS            = 2\nuint32 STATUS_FAILED_TRANS_UNKNOWN_BEHAVIOR = 4\nuint32 STATUS_FAILED_TRANS_ILLEGAL_BEHAVIOR = 8\nuint32 STATUS_FAILED_TRANS_COM_POS          = 16\nuint32 STATUS_FAILED_TRANS_COM_VEL          = 32\nuint32 STATUS_FAILED_TRANS_VEL              = 64\nuint32 STATUS_WARNING_AUTO_TRANS            = 128\nuint32 STATUS_ERROR_FALLING                 = 256\n\nuint32 status_flags  # can be one of above\n\nint32 trans_from_behavior_index  # use this as a parm to get_behavior_at_index() to get behavior string\nint32 trans_to_behavior_index  # use this as a parm to get_behavior_at_index() to get behavior string\n";
  static final int STATUS_OK = 0;
  static final int STATUS_TRANSITION_IN_PROGRESS = 1;
  static final int STATUS_TRANSITION_SUCCESS = 2;
  static final int STATUS_FAILED_TRANS_UNKNOWN_BEHAVIOR = 4;
  static final int STATUS_FAILED_TRANS_ILLEGAL_BEHAVIOR = 8;
  static final int STATUS_FAILED_TRANS_COM_POS = 16;
  static final int STATUS_FAILED_TRANS_COM_VEL = 32;
  static final int STATUS_FAILED_TRANS_VEL = 64;
  static final int STATUS_WARNING_AUTO_TRANS = 128;
  static final int STATUS_ERROR_FALLING = 256;
  int getStatusFlags();
  void setStatusFlags(int value);
  int getTransFromBehaviorIndex();
  void setTransFromBehaviorIndex(int value);
  int getTransToBehaviorIndex();
  void setTransToBehaviorIndex(int value);
}
