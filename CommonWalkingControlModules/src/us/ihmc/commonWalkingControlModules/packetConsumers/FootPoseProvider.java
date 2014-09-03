package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePose;

public interface FootPoseProvider
{
   public abstract boolean checkForNewPose(RobotSide robotSide);

   public abstract RobotSide checkForNewPose();

   public abstract FramePose getDesiredFootPose(RobotSide robotSide);
}
