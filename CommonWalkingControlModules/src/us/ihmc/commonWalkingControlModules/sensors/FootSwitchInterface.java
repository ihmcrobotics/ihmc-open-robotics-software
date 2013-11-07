package us.ihmc.commonWalkingControlModules.sensors;


public interface FootSwitchInterface
{
   public abstract boolean hasFootHitGround();
   
   public abstract double computeFootLoadPercentage();

   public void reset();
}
