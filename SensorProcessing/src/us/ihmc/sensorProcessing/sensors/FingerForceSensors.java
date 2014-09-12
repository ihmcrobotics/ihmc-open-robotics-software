package us.ihmc.sensorProcessing.sensors;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.partNames.FingerName;
import us.ihmc.utilities.math.geometry.FrameVector;

public interface FingerForceSensors
{
   public abstract FrameVector getFingerForce(RobotSide robotSide, FingerName fingerName);
}
