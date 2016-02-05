package gazebo_msgs;

public interface GetJointPropertiesResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/GetJointPropertiesResponse";
  static final java.lang.String _DEFINITION = "# joint type\nuint8 type\nuint8 REVOLUTE    = 0                # single DOF\nuint8 CONTINUOUS  = 1                # single DOF (revolute w/o joints)\nuint8 PRISMATIC   = 2                # single DOF\nuint8 FIXED       = 3                # 0 DOF\nuint8 BALL        = 4                # 3 DOF\nuint8 UNIVERSAL   = 5                # 2 DOF\n# dynamics properties\nfloat64[] damping\n# joint state\nfloat64[] position\nfloat64[] rate\n# service return status\nbool success                         # return true if get successful\nstring status_message                # comments if available";
  static final byte REVOLUTE = 0;
  static final byte CONTINUOUS = 1;
  static final byte PRISMATIC = 2;
  static final byte FIXED = 3;
  static final byte BALL = 4;
  static final byte UNIVERSAL = 5;
  byte getType();
  void setType(byte value);
  double[] getDamping();
  void setDamping(double[] value);
  double[] getPosition();
  void setPosition(double[] value);
  double[] getRate();
  void setRate(double[] value);
  boolean getSuccess();
  void setSuccess(boolean value);
  java.lang.String getStatusMessage();
  void setStatusMessage(java.lang.String value);
}
