package us.ihmc.sensorProcessing.model;


public interface RobotMotionStatusChangedListener
{
   public void robotMotionStatusHasChanged(RobotMotionStatus newStatus, double time);
}
