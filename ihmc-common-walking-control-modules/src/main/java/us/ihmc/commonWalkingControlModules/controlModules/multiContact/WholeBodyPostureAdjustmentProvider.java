package us.ihmc.commonWalkingControlModules.controlModules.multiContact;

public interface WholeBodyPostureAdjustmentProvider
{
   boolean isEnabled();

   double getDesiredJointPositionOffset(String jointName);

   double getDesiredJointVelocityOffset(String jointName);
}
