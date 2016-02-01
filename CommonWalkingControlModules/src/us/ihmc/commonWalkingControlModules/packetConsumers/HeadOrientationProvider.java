package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface HeadOrientationProvider
{
   public abstract boolean isNewLookAtInformationAvailable();

   public abstract boolean isNewHeadOrientationInformationAvailable();

   public abstract FrameOrientation getDesiredHeadOrientation();

   public abstract FramePoint getLookAtPoint();

   public abstract ReferenceFrame getHeadOrientationExpressedInFrame();

   public abstract double getTrajectoryTime();
}