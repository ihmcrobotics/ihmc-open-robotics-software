package us.ihmc.commonWalkingControlModules.sensors;

public interface ToeSwitch extends FootSwitchInterface
{
   public abstract boolean hasToeHitGround();
   
   public void resetToeSwitch();
}
