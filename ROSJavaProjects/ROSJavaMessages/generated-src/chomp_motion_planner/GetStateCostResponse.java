package chomp_motion_planner;

public interface GetStateCostResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "chomp_motion_planner/GetStateCostResponse";
  static final java.lang.String _DEFINITION = "# True if the cost computation was valid\nbool valid\n# The cost corresponding to each link\nfloat64[] costs";
  boolean getValid();
  void setValid(boolean value);
  double[] getCosts();
  void setCosts(double[] value);
}
