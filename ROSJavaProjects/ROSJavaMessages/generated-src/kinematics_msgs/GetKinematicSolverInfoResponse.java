package kinematics_msgs;

public interface GetKinematicSolverInfoResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "kinematics_msgs/GetKinematicSolverInfoResponse";
  static final java.lang.String _DEFINITION = "kinematics_msgs/KinematicSolverInfo kinematic_solver_info";
  kinematics_msgs.KinematicSolverInfo getKinematicSolverInfo();
  void setKinematicSolverInfo(kinematics_msgs.KinematicSolverInfo value);
}
