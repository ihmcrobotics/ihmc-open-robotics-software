package us.ihmc.commonWalkingControlModules.packetProviders;

import us.ihmc.utilities.robotSide.RobotSide;

public interface ControlStatusProducer
{
   public void notifyHandTrajectoryInfeasible(RobotSide robotSide);
}
