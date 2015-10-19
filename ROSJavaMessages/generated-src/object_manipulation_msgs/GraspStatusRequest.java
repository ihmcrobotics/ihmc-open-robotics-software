package object_manipulation_msgs;

public interface GraspStatusRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "object_manipulation_msgs/GraspStatusRequest";
  static final java.lang.String _DEFINITION = "# asks if a grasp is currently active\n# used to query if the object is still detected as present in the hand\n# (using joint angles, forces sensors or any other proprioceptive methods)\n\n# the name of the arm being used is not in here, as this will be sent to a specific action server\n# for each arm\n\n# the grasp that we are checking \nobject_manipulation_msgs/Grasp grasp\n\n";
  object_manipulation_msgs.Grasp getGrasp();
  void setGrasp(object_manipulation_msgs.Grasp value);
}
