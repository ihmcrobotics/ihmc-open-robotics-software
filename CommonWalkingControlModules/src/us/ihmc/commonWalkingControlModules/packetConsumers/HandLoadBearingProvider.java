package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.utilities.robotSide.RobotSide;

public interface HandLoadBearingProvider
{
   public abstract boolean checkForNewInformation(RobotSide robotSide);
   public abstract boolean hasLoadBearingBeenRequested(RobotSide robotSide);
}