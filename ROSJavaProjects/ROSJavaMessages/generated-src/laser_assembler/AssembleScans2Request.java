package laser_assembler;

public interface AssembleScans2Request extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "laser_assembler/AssembleScans2Request";
  static final java.lang.String _DEFINITION = "# The time interval on which we want to aggregate scans\ntime begin\n# The end of the interval on which we want to assemble scans or clouds\ntime end\n";
  org.ros.message.Time getBegin();
  void setBegin(org.ros.message.Time value);
  org.ros.message.Time getEnd();
  void setEnd(org.ros.message.Time value);
}
