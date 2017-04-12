package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

public interface LowLevelJointDataReadOnly
{
   public abstract boolean hasControlMode();
   public abstract boolean hasDesiredTorque();
   public abstract boolean hasDesiredPosition();
   public abstract boolean hasDesiredVelocity();
   public abstract boolean hasDesiredAcceleration();
   public abstract boolean hasDesiredCurrent();
   public abstract LowLevelJointControlMode getControlMode();
   public abstract double getDesiredTorque();
   public abstract double getDesiredPosition();
   public abstract double getDesiredVelocity();
   public abstract double getDesiredAcceleration();
   public abstract double getDesiredCurrent();
   public abstract boolean pollResetIntegratorsRequest();
   public abstract boolean peekResetIntegratorsRequest();
}