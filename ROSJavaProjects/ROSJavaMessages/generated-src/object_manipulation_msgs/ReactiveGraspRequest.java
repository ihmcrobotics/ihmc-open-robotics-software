package object_manipulation_msgs;

public interface ReactiveGraspRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/ReactiveGraspRequest";
  static final java.lang.String _DEFINITION = "# the goal of the reactive grasp\nReactiveGraspGoal goal\n\n";
  object_manipulation_msgs.ReactiveGraspGoal getGoal();
  void setGoal(object_manipulation_msgs.ReactiveGraspGoal value);
}
