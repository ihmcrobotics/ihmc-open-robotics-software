package us.ihmc.commonWalkingControlModules.packetProviders;

import us.ihmc.robotics.robotSide.RobotSide;

public interface ControlStatusProducer
{
   public void notifyHandTrajectoryInfeasible(RobotSide robotSide);
}
