package ihmc_msgs;

public interface WeightMatrix3DRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/WeightMatrix3DRosMessage";
  static final java.lang.String _DEFINITION = "## WeightMatrix3DRosMessage\n# \n\n# The Id of the reference frame defining the weight frame. This reference frame defines the x axis,y\n# axis, z axis for the weights. This frame is optional. It is preferable to provide it when possible,\n# but when it is absent, i.e. equal to {@code 0L}, the weight matrix will then be generated regardless\n# to what frame is it used in.\nint64 weight_frame_id\n\n# Specifies the qp weight for the x-axis, If set to NaN the controller will use the default weight for\n# this axis. The weight is NaN by default.\nfloat64 x_weight\n\n# Specifies the qp weight for the y-axis, If set to NaN the controller will use the default weight for\n# this axis. The weight is NaN by default.\nfloat64 y_weight\n\n# Specifies the qp weight for the z-axis, If set to NaN the controller will use the default weight for\n# this axis. The weight is NaN by default.\nfloat64 z_weight\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  long getWeightFrameId();
  void setWeightFrameId(long value);
  double getXWeight();
  void setXWeight(double value);
  double getYWeight();
  void setYWeight(double value);
  double getZWeight();
  void setZWeight(double value);
  long getUniqueId();
  void setUniqueId(long value);
}
