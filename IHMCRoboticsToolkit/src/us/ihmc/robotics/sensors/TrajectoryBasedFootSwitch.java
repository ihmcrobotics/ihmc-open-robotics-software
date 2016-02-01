package us.ihmc.robotics.sensors;

public interface TrajectoryBasedFootSwitch extends FootSwitchProvider
{
   public boolean isSwinging();
   public void setIsSwinging(boolean isSwinging);
}
