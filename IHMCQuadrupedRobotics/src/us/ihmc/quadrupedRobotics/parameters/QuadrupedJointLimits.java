package us.ihmc.quadrupedRobotics.parameters;

public interface QuadrupedJointLimits
{
   public double getJointPositionLowerLimit(String jointName);
   public double getJointPositionUpperLimit(String jointName);
   public double getJointSoftPositionLowerLimit(String jointName);
   public double getJointSoftPositionUpperLimit(String jointName);
   public double getJointEffortLimit(String jointName);
   public double getJointVelocityLimit(String jointName);
   public double getJointAccelerationLimit(String jointName);
}