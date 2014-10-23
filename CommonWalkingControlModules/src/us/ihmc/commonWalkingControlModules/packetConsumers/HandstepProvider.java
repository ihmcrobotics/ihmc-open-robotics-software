package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.desiredFootStep.Handstep;
import us.ihmc.utilities.robotSide.RobotSide;

public interface HandstepProvider
{
   public abstract Handstep getDesiredHandstep(RobotSide robotSide);

   public abstract boolean checkForNewHandstep(RobotSide robotSide);
}
