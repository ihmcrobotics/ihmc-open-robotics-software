package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.robotics.geometry.ReferenceFrameMismatchException;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FrameBasedMessage
{
   public static void checkIfTrajectoryFrameIdsMatch(long frameId, ReferenceFrame referenceFrame)
   {
      if (frameId != referenceFrame.getNameBasedHashCode() && frameId != referenceFrame.getAdditionalNameBasedHashCode())
      {
         String msg = "Argument's hashcode " + referenceFrame + " " + referenceFrame.getNameBasedHashCode() + " does not match " + frameId;
         throw new ReferenceFrameMismatchException(msg);
      }
   }

   public static void checkIfFrameIdsMatch(long frameId, long otherReferenceFrameId)
   {
      if (frameId != otherReferenceFrameId)
      {
         String msg = "Argument's hashcode " + otherReferenceFrameId + " does not match " + frameId;
         throw new ReferenceFrameMismatchException(msg);
      }
   }
}
