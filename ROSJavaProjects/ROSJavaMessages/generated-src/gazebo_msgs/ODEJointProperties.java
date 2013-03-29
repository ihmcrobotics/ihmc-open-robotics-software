package gazebo_msgs;

public interface ODEJointProperties extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "gazebo_msgs/ODEJointProperties";
  static final java.lang.String _DEFINITION = "# access to low level joint properties, change these at your own risk\nfloat64[] damping             # joint damping\nfloat64[] hiStop              # joint limit\nfloat64[] loStop              # joint limit\nfloat64[] erp                 # set joint erp\nfloat64[] cfm                 # set joint cfm\nfloat64[] stop_erp            # set joint erp for joint limit \"contact\" joint\nfloat64[] stop_cfm            # set joint cfm for joint limit \"contact\" joint\nfloat64[] fudge_factor        # joint fudge_factor applied at limits, see ODE manual for info.\nfloat64[] fmax                # ode joint param fmax\nfloat64[] vel                 # ode joint param vel\n";
  double[] getDamping();
  void setDamping(double[] value);
  double[] getHiStop();
  void setHiStop(double[] value);
  double[] getLoStop();
  void setLoStop(double[] value);
  double[] getErp();
  void setErp(double[] value);
  double[] getCfm();
  void setCfm(double[] value);
  double[] getStopErp();
  void setStopErp(double[] value);
  double[] getStopCfm();
  void setStopCfm(double[] value);
  double[] getFudgeFactor();
  void setFudgeFactor(double[] value);
  double[] getFmax();
  void setFmax(double[] value);
  double[] getVel();
  void setVel(double[] value);
}
