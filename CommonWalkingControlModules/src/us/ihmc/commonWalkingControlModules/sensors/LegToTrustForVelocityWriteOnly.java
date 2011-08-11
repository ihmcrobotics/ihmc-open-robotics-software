package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.robotSide.RobotSide;

public interface LegToTrustForVelocityWriteOnly
{
   public abstract void setLegToTrustForVelocity(RobotSide robotSide);
   public abstract void setSupportLeg(RobotSide robotSide);
   public abstract void setLegToUseForCOMOffset(RobotSide robotSide);
}
