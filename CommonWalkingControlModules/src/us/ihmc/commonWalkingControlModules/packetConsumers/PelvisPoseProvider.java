package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;

public interface PelvisPoseProvider
{
   public abstract boolean checkForNewPosition();

   public abstract boolean checkForNewOrientation();

   public abstract FramePoint getDesiredPelvisPosition();

   public abstract FrameOrientation getDesiredPelvisOrientation();

   public abstract double getTrajectoryTime();
}