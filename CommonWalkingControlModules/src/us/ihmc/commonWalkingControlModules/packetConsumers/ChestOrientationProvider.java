package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface ChestOrientationProvider
{
   public abstract ReferenceFrame getChestOrientationExpressedInFrame();

   public abstract boolean checkForNewChestOrientation();
   
   public abstract boolean checkForHomeOrientation();

   public abstract FrameOrientation getDesiredChestOrientation();

   public abstract double getTrajectoryTime();
}