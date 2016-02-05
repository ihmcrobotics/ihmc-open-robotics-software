package test_ros;

public interface Composite extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "test_ros/Composite";
  static final java.lang.String _DEFINITION = "# composite message. required for testing import calculation in generators\nCompositeA a\nCompositeB b\n";
  test_ros.CompositeA getA();
  void setA(test_ros.CompositeA value);
  test_ros.CompositeB getB();
  void setB(test_ros.CompositeB value);
}
