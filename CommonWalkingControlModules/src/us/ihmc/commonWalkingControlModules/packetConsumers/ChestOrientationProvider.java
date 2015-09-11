package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.math.trajectories.WaypointOrientationTrajectoryData;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface ChestOrientationProvider
{
   public abstract ReferenceFrame getChestOrientationExpressedInFrame();

   public abstract boolean checkForNewChestOrientation();

   public abstract boolean checkForHomeOrientation();

   public abstract boolean checkForNewChestOrientationWithWaypoints();

   public abstract FrameOrientation getDesiredChestOrientation();

   public abstract WaypointOrientationTrajectoryData getDesiredChestOrientationWithWaypoints();

   public abstract double getTrajectoryTime();
}