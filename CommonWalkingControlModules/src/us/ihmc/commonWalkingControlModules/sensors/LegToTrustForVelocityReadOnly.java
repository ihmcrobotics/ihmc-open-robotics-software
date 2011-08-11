package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.robotSide.RobotSide;

public interface LegToTrustForVelocityReadOnly
{
   public abstract RobotSide getLegToTrustForVelocity();
   public abstract RobotSide getSupportLeg();
   public abstract RobotSide getLegToUseForCOMOffset();
}
