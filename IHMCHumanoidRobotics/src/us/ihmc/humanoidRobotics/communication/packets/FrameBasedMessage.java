package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FrameBasedMessage
{

   long getTrajectoryReferenceFrameId();
   long getDataReferenceFrameId();
   void setTrajectoryReferenceFrameId(long trajedtoryReferenceFrameId);
   void setTrajectoryReferenceFrameId(ReferenceFrame trajectoryReferenceFrame);
   void setDataReferenceFrameId(long expressedInReferenceFrameId);
   void setDataReferenceFrameId(ReferenceFrame expressedInReferenceFrame);
}
