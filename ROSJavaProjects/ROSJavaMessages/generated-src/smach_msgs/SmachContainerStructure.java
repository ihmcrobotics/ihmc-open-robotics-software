package smach_msgs;

public interface SmachContainerStructure extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "smach_msgs/SmachContainerStructure";
  static final java.lang.String _DEFINITION = "Header header\n\n# The path to this node in the server\nstring path\n\n# The children of this node\nstring[] children\n\n# The outcome edges\n# Each index across these arrays denote one edge\nstring[] internal_outcomes\nstring[] outcomes_from\nstring[] outcomes_to\n\n# The potential outcomes from this container\nstring[] container_outcomes\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.lang.String getPath();
  void setPath(java.lang.String value);
  java.util.List<java.lang.String> getChildren();
  void setChildren(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getInternalOutcomes();
  void setInternalOutcomes(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getOutcomesFrom();
  void setOutcomesFrom(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getOutcomesTo();
  void setOutcomesTo(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getContainerOutcomes();
  void setContainerOutcomes(java.util.List<java.lang.String> value);
}
