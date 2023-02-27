package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.robotics.sensors.FootSwitchInterface;

public interface QuadrupedFootSwitchInterface extends FootSwitchInterface
{
   public void setFootContactState(boolean hasFootHitGround);

   public void trustFootSwitchInSwing(boolean trustFootSwitchInSwing);

   public void trustFootSwitchInSupport(boolean trustFootSwitchInSupport);
}
