package us.ihmc.acsell.hardware.configuration;

public interface AcsellAnkleKinematicParameters
{

   public abstract boolean useJacobianComputedFromMotors();
   public abstract boolean isJacobianFromJointAnglesComputationPerformed();
   public abstract boolean isJacobianFromMotorAnglesComputationPerformed();
   
   public abstract double getN();
   public abstract double[] getM2Params();
   public abstract double[] getM1Params();
   public abstract double[] getJITY_FromJointParams();
   public abstract double[] getJITX_FromJointParams();
   public abstract double[] getJITY_FromMotorParams();
   public abstract double[] getJITX_FromMotorParams();
   public abstract double[] getYParams();
   public abstract double[] getXParams();
   
   public abstract AcsellAnklePhysicalParameters getLeftAnkleLeftParams();
   public abstract AcsellAnklePhysicalParameters getRightAnkleLeftParams();
   public abstract AcsellAnklePhysicalParameters getLeftAnkleRightParams();
   public abstract AcsellAnklePhysicalParameters getRightAnkleRightParams();

}
