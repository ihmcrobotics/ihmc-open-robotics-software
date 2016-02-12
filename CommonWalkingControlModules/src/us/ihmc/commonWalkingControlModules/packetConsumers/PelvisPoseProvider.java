package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface PelvisPoseProvider
{
   public abstract boolean checkForNewPosition();

   public abstract boolean checkForNewOrientation();
   
   public abstract boolean checkForHomePosition();

   public abstract boolean checkForHomeOrientation();

   public abstract FramePoint getDesiredPelvisPosition(ReferenceFrame supportFrame);

   public abstract FrameOrientation getDesiredPelvisOrientation(ReferenceFrame desiredPelvisFrame);

   public abstract double getTrajectoryTime();
   
   public abstract void clearOrientation();
   
   public abstract void clearPosition();
}