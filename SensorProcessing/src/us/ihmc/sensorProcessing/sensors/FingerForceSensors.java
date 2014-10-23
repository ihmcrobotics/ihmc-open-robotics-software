package us.ihmc.sensorProcessing.sensors;

import us.ihmc.utilities.humanoidRobot.partNames.FingerName;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.robotSide.RobotSide;

public interface FingerForceSensors
{
   public abstract FrameVector getFingerForce(RobotSide robotSide, FingerName fingerName);
}
