package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3TrajectoryMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SE3Message extends AbstractSE3TrajectoryMessage<SE3Message>
{
   public SE3Message(int numberOfPoints, ReferenceFrame trajectoryFrame)
   {
      super(numberOfPoints);
      getFrameInformation().setTrajectoryReferenceFrame(trajectoryFrame);
   }
}
