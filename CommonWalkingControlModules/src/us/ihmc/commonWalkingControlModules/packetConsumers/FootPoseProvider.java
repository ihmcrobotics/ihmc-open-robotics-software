package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.robotSide.RobotSide;

public interface FootPoseProvider
{
   public abstract boolean checkForNewPose(RobotSide robotSide);

   public abstract RobotSide checkForNewPose();

   public abstract FramePose getDesiredFootPose(RobotSide robotSide);

   public abstract double getTrajectoryTime();

   public abstract void clear();
}
