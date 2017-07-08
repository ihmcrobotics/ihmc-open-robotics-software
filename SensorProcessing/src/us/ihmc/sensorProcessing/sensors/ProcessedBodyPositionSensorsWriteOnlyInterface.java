package us.ihmc.sensorProcessing.sensors;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector3D;

public interface ProcessedBodyPositionSensorsWriteOnlyInterface
{
   public abstract void setBodyPosition(FramePoint bodyPosition);
   public abstract void setBodyVelocity(FrameVector3D bodyVelocity);
}
