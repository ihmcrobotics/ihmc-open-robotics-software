package atlas_msgs;

public interface AtlasBehaviorStepParams extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasBehaviorStepParams";
  static final java.lang.String _DEFINITION = "# parameters for single_step behavior\natlas_msgs/AtlasBehaviorStepData desired_step\nbool use_demo_walk\n";
  atlas_msgs.AtlasBehaviorStepData getDesiredStep();
  void setDesiredStep(atlas_msgs.AtlasBehaviorStepData value);
  boolean getUseDemoWalk();
  void setUseDemoWalk(boolean value);
}
