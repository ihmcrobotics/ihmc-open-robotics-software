package us.ihmc.sensorProcessing.sensors;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;

public interface ProcessedBodyPositionSensorsWriteOnlyInterface
{
   public abstract void setBodyPosition(FramePoint3D bodyPosition);
   public abstract void setBodyVelocity(FrameVector3D bodyVelocity);
}
