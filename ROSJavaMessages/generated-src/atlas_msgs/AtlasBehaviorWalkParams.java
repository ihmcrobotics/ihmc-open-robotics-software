package atlas_msgs;

public interface AtlasBehaviorWalkParams extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorWalkParams";
  static final java.lang.String _DEFINITION = "# multi_step walking trajectory parameters\natlas_msgs/AtlasBehaviorStepData[4] step_queue\nbool use_demo_walk\n";
  java.util.List<atlas_msgs.AtlasBehaviorStepData> getStepQueue();
  void setStepQueue(java.util.List<atlas_msgs.AtlasBehaviorStepData> value);
  boolean getUseDemoWalk();
  void setUseDemoWalk(boolean value);
}
