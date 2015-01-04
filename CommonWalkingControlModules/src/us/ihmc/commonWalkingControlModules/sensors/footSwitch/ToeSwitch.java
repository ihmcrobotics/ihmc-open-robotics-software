package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

public interface ToeSwitch extends FootSwitchInterface
{
   public abstract boolean hasToeHitGround();
   
   public void resetToeSwitch();
}
