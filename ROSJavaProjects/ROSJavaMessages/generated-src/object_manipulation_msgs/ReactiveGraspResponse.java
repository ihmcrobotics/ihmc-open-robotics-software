package object_manipulation_msgs;

public interface ReactiveGraspResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/ReactiveGraspResponse";
  static final java.lang.String _DEFINITION = "\n# the result of the reactive grasp\nReactiveGraspResult result";
  object_manipulation_msgs.ReactiveGraspResult getResult();
  void setResult(object_manipulation_msgs.ReactiveGraspResult value);
}
