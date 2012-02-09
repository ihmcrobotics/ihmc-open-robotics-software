package com.yobotics.simulationconstructionset.processedSensors;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;

public interface ProcessedBodyPositionSensorsWriteOnlyInterface
{
   public abstract void setBodyPosition(FramePoint bodyPosition);
   public abstract void setBodyVelocity(FrameVector bodyVelocity);
}
