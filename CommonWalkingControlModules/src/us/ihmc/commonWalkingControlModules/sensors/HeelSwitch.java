package us.ihmc.commonWalkingControlModules.sensors;

public interface HeelSwitch extends FootSwitchInterface
{
   public abstract boolean hasHeelHitGround();
   
   public void resetHeelSwitch();
}
