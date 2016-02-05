package test_ros;

public interface Embed extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/Embed";
  static final java.lang.String _DEFINITION = "#for rostopic tests\nSimple simple\nArrays arrays\n";
  test_ros.Simple getSimple();
  void setSimple(test_ros.Simple value);
  test_ros.Arrays getArrays();
  void setArrays(test_ros.Arrays value);
}
