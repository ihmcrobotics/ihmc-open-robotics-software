package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface PelvisPoseProvider
{
   public abstract boolean checkForNewPosition();

   public abstract boolean checkForNewOrientation();

   public abstract FramePoint getDesiredPelvisPosition(ReferenceFrame supportFrame);

   public abstract FrameOrientation getDesiredPelvisOrientation(ReferenceFrame desiredPelvisFrame);

   public abstract double getTrajectoryTime();
}