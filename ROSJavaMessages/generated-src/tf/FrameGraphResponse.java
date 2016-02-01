package tf;

public interface FrameGraphResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "tf/FrameGraphResponse";
  static final java.lang.String _DEFINITION = "string dot_graph";
  java.lang.String getDotGraph();
  void setDotGraph(java.lang.String value);
}
