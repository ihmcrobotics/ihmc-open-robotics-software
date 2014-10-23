package us.ihmc.sensorProcessing.stateEstimation;

import us.ihmc.utilities.robotSide.RobotSide;

public interface LegToTrustForVelocityWriteOnly
{
   public abstract void setLegToTrustForVelocity(RobotSide robotSide, boolean trust);
   public abstract void setSupportLeg(RobotSide robotSide);
   public abstract void setLegToUseForCOMOffset(RobotSide robotSide);
}
