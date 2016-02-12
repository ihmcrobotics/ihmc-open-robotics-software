package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.robotics.geometry.FrameOrientation;

public interface ChestOrientationProvider
{
   public abstract boolean checkForNewChestOrientation();

   public abstract boolean checkForHomeOrientation();

   public abstract FrameOrientation getDesiredChestOrientation();

   public abstract double getTrajectoryTime();
}