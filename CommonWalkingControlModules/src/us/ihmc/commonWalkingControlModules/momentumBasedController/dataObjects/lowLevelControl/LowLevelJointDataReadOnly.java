package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.lowLevelControl;

public interface LowLevelJointDataReadOnly
{
   public boolean hasControlMode();
   public boolean hasDesiredTorque();
   public boolean hasDesiredPosition();
   public boolean hasDesiredVelocity();
   public boolean hasDesiredAcceleration();
   public LowLevelJointControlMode getControlMode();
   public double getDesiredTorque();
   public double getDesiredPosition();
   public double getDesiredVelocity();
   public double getDesiredAcceleration();
   public boolean pollResetIntegratorsRequest();
   public boolean peekResetIntegratorsRequest();
}