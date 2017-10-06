package us.ihmc.valkyrie.kinematics.transmissions;

public interface PushrodTransmissionJacobian
{
   public abstract void setUseFuteks(boolean useFuteks);
   public abstract void computeJacobian(double[][] jacobian, double topJointAngle, double bottomJointAngle);
}
