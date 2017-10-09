package us.ihmc.sensorProcessing.sensors;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.robotics.robotSide.RobotSide;

public interface FingerForceSensors
{
   public abstract FrameVector3D getFingerForce(RobotSide robotSide, FingerName fingerName);
}
