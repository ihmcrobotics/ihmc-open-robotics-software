package atlas_msgs;

public interface AtlasBehaviorWalkFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorWalkFeedback";
  static final java.lang.String _DEFINITION = "# mirrored from AtlasControlTypes.h\nfloat64 t_step_rem\nuint32 current_step_index\nuint32 next_step_index_needed\nuint32 status_flags    # use AtlasBeahviorFeedback/status_flags\natlas_msgs/AtlasBehaviorStepData[4] step_queue_saturated # 4 is hardcoded in AtlasSimInterface library.\n";
  double getTStepRem();
  void setTStepRem(double value);
  int getCurrentStepIndex();
  void setCurrentStepIndex(int value);
  int getNextStepIndexNeeded();
  void setNextStepIndexNeeded(int value);
  int getStatusFlags();
  void setStatusFlags(int value);
  java.util.List<atlas_msgs.AtlasBehaviorStepData> getStepQueueSaturated();
  void setStepQueueSaturated(java.util.List<atlas_msgs.AtlasBehaviorStepData> value);
}
