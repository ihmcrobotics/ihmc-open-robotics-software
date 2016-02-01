package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

public interface HeelSwitch extends FootSwitchInterface
{
   public abstract boolean hasHeelHitGround();
   
   public void resetHeelSwitch();
}
