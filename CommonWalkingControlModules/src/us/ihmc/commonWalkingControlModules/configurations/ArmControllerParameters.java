package us.ihmc.commonWalkingControlModules.configurations;

public interface ArmControllerParameters
{
   public abstract double getKpAllArmJoints();
   public abstract double getZetaAllArmJoints();
   public abstract double getMaxAccelerationAllArmJoints();
   public abstract double getMaxJerkAllArmJoints();
}
