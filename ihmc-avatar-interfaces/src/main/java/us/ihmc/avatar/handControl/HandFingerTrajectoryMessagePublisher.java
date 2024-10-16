package us.ihmc.avatar.handControl;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.robotics.robotSide.RobotSide;

public interface HandFingerTrajectoryMessagePublisher
{
   public abstract void sendFingerTrajectoryMessage(RobotSide robotSide, TDoubleArrayList desiredPositions, TDoubleArrayList trajectoryTimes);
}
