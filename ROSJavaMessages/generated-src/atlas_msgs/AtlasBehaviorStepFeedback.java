package atlas_msgs;

public interface AtlasBehaviorStepFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorStepFeedback";
  static final java.lang.String _DEFINITION = "# mirrored from AtlasControlTypes.h\nuint32 STEP_SUBSTATE_SWAYING = 0  # Feet are in double support. This flag does not latch. Only one of STEP_SUBSTATE_SWAYING or STEP_SUBSTATE_STEPPING will be set at any given time.\nuint32 STEP_SUBSTATE_STEPPING = 1 # Actively stepping; one foot is in the air. This flag does not latch.\n\nuint32 status_flags    # use AtlasBeahviorFeedback/status_flags\nfloat64 t_step_rem\nuint32 current_step_index\nuint32 next_step_index_needed\natlas_msgs/AtlasBehaviorStepData desired_step_saturated\n";
  static final int STEP_SUBSTATE_SWAYING = 0;
  static final int STEP_SUBSTATE_STEPPING = 1;
  int getStatusFlags();
  void setStatusFlags(int value);
  double getTStepRem();
  void setTStepRem(double value);
  int getCurrentStepIndex();
  void setCurrentStepIndex(int value);
  int getNextStepIndexNeeded();
  void setNextStepIndexNeeded(int value);
  atlas_msgs.AtlasBehaviorStepData getDesiredStepSaturated();
  void setDesiredStepSaturated(atlas_msgs.AtlasBehaviorStepData value);
}
