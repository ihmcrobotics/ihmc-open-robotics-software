package atlas_msgs;

public interface AtlasState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "atlas_msgs/AtlasState";
  static final java.lang.String _DEFINITION = "#\n# This message has been carefully constructed to be less\n# than 1500 in size when serialized, to accommodate transfer\n# UDP.\n#\n# testing everything a robot needs\nHeader header\n\n# Default joint indices used when publishing the\n# JointCommands joint_states topic below\n# For exmaple, if you subscribe to this message, then\n# msg.joint_states.position[atlas_msgs::AtlasStates::back_lbz] gives back\n# the position of the back_lbz.\nint32 back_lbz  = 0\nint32 back_mby  = 1\nint32 back_ubx  = 2\nint32 neck_ay   = 3\nint32 l_leg_uhz = 4\nint32 l_leg_mhx = 5\nint32 l_leg_lhy = 6\nint32 l_leg_kny = 7\nint32 l_leg_uay = 8\nint32 l_leg_lax = 9\nint32 r_leg_uhz = 10\nint32 r_leg_mhx = 11\nint32 r_leg_lhy = 12\nint32 r_leg_kny = 13\nint32 r_leg_uay = 14\nint32 r_leg_lax = 15\nint32 l_arm_usy = 16\nint32 l_arm_shx = 17\nint32 l_arm_ely = 18\nint32 l_arm_elx = 19\nint32 l_arm_uwy = 20\nint32 l_arm_mwx = 21\nint32 r_arm_usy = 22\nint32 r_arm_shx = 23\nint32 r_arm_ely = 24\nint32 r_arm_elx = 25\nint32 r_arm_uwy = 26\nint32 r_arm_mwx = 27\n\n# repeating data from osrf_msgs/JointCommands as joint_states\nfloat32[] position\nfloat32[] velocity\nfloat32[] effort\nfloat32[] kp_position\nfloat32[] ki_position\nfloat32[] kd_position\nfloat32[] kp_velocity\nfloat32[] i_effort_min\nfloat32[] i_effort_max\n\nuint8[] k_effort       # k_effort can be an unsigned int 8value from 0 to 255, \n                       # at run time, a double between 0 and 1 is obtained\n                       # by dividing by 255.0d.\n\n\n#sensor_msgs/Imu imu \ngeometry_msgs/Quaternion orientation\ngeometry_msgs/Vector3 angular_velocity\ngeometry_msgs/Vector3 linear_acceleration\n\n#atlas_msgs/ForceTorqueSensors force_torque_sensors\ngeometry_msgs/Wrench l_foot\ngeometry_msgs/Wrench r_foot\ngeometry_msgs/Wrench l_hand\ngeometry_msgs/Wrench r_hand\n";
  static final int back_lbz = 0;
  static final int back_mby = 1;
  static final int back_ubx = 2;
  static final int neck_ay = 3;
  static final int l_leg_uhz = 4;
  static final int l_leg_mhx = 5;
  static final int l_leg_lhy = 6;
  static final int l_leg_kny = 7;
  static final int l_leg_uay = 8;
  static final int l_leg_lax = 9;
  static final int r_leg_uhz = 10;
  static final int r_leg_mhx = 11;
  static final int r_leg_lhy = 12;
  static final int r_leg_kny = 13;
  static final int r_leg_uay = 14;
  static final int r_leg_lax = 15;
  static final int l_arm_usy = 16;
  static final int l_arm_shx = 17;
  static final int l_arm_ely = 18;
  static final int l_arm_elx = 19;
  static final int l_arm_uwy = 20;
  static final int l_arm_mwx = 21;
  static final int r_arm_usy = 22;
  static final int r_arm_shx = 23;
  static final int r_arm_ely = 24;
  static final int r_arm_elx = 25;
  static final int r_arm_uwy = 26;
  static final int r_arm_mwx = 27;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float[] getPosition();
  void setPosition(float[] value);
  float[] getVelocity();
  void setVelocity(float[] value);
  float[] getEffort();
  void setEffort(float[] value);
  float[] getKpPosition();
  void setKpPosition(float[] value);
  float[] getKiPosition();
  void setKiPosition(float[] value);
  float[] getKdPosition();
  void setKdPosition(float[] value);
  float[] getKpVelocity();
  void setKpVelocity(float[] value);
  float[] getIEffortMin();
  void setIEffortMin(float[] value);
  float[] getIEffortMax();
  void setIEffortMax(float[] value);
  org.jboss.netty.buffer.ChannelBuffer getKEffort();
  void setKEffort(org.jboss.netty.buffer.ChannelBuffer value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  geometry_msgs.Vector3 getAngularVelocity();
  void setAngularVelocity(geometry_msgs.Vector3 value);
  geometry_msgs.Vector3 getLinearAcceleration();
  void setLinearAcceleration(geometry_msgs.Vector3 value);
  geometry_msgs.Wrench getLFoot();
  void setLFoot(geometry_msgs.Wrench value);
  geometry_msgs.Wrench getRFoot();
  void setRFoot(geometry_msgs.Wrench value);
  geometry_msgs.Wrench getLHand();
  void setLHand(geometry_msgs.Wrench value);
  geometry_msgs.Wrench getRHand();
  void setRHand(geometry_msgs.Wrench value);
}
