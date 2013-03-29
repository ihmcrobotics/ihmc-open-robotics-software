package gazebo;

public interface GetJointProperties extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo/GetJointProperties";
  static final java.lang.String _DEFINITION = "#This message is deprecated.  Please use the version in gazebo_msgs instead.\n\nstring joint_name                    # name of joint\n---\n# joint type\nuint8 type\nuint8 REVOLUTE    = 0                # single DOF\nuint8 CONTINUOUS  = 1                # single DOF (revolute w/o joints)\nuint8 PRISMATIC   = 2                # single DOF\nuint8 FIXED       = 3                # 0 DOF\nuint8 BALL        = 4                # 3 DOF\nuint8 UNIVERSAL   = 5                # 2 DOF\n# dynamics properties\nfloat64[] damping\n# joint state\nfloat64[] position\nfloat64[] rate\n# service return status\nbool success                         # return true if get successful\nstring status_message                # comments if available\n";
}
