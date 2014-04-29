package us.ihmc.commonWalkingControlModules.sensors;

import us.ihmc.utilities.math.geometry.FramePoint2d;


public interface FootSwitchInterface
{
   public abstract boolean hasFootHitGround();
   
   public abstract double computeFootLoadPercentage();
   public abstract void computeAndPackCoP(FramePoint2d copToPack);
   
   public abstract void setSwingTrajectoryWasReplanned(boolean value);

   public void reset();
}
