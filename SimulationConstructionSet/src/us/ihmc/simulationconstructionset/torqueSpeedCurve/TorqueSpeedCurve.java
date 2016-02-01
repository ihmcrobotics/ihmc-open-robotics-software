package us.ihmc.simulationconstructionset.torqueSpeedCurve;

public interface TorqueSpeedCurve
{
   public double limitTorque(double torque, double speed);
}
