package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FrameBasedMessage
{

   long getTrajectoryReferenceFrameId();
   long getExpressedInReferenceFrameId();
   void setTrajectoryReferenceFrameId(long trajedtoryReferenceFrameId);
   void setTrajectoryReferenceFrameId(ReferenceFrame trajectoryReferenceFrame);
   void setExpressedInReferenceFrameId(long expressedInReferenceFrameId);
   void setExpressedInReferenceFrameId(ReferenceFrame expressedInReferenceFrame);
}
