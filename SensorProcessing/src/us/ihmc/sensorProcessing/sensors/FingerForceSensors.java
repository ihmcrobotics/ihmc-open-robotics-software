package us.ihmc.sensorProcessing.sensors;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;

public interface FingerForceSensors
{
   public abstract FrameVector getFingerForce(RobotSide robotSide, FingerName fingerName);
}
