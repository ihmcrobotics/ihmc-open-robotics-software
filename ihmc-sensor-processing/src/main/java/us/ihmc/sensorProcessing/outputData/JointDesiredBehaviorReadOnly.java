package us.ihmc.sensorProcessing.outputData;

/**
 * Parameter class that defines desired joint behavior.
 * <p>
 * This class is meant to contain parameters used in the joint level control laws
 * that are used to track controller desired joint outputs.
 * </p>
 */
public interface JointDesiredBehaviorReadOnly
{
   public JointDesiredControlMode getControlMode();

   public double getStiffness();

   public double getDamping();

   public double getMasterGain();

   public double getVelocityScaling();

   public double getMaxPositionError();

   public double getMaxVelocityError();
}