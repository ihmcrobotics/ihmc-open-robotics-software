package us.ihmc.commonWalkingControlModules.packetProviders;

import us.ihmc.robotSide.RobotSide;

public interface ControlStatusProducer
{
   public void notifyHandTrajectoryInfeasible(RobotSide robotSide);
}
