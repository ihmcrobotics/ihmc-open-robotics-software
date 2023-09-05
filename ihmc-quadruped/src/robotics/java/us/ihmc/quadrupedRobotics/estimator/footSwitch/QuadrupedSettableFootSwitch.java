package us.ihmc.quadrupedRobotics.estimator.footSwitch;

import us.ihmc.commonWalkingControlModules.sensors.footSwitch.SettableFootSwitch;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.yoVariables.registry.YoRegistry;

public class QuadrupedSettableFootSwitch extends SettableFootSwitch implements QuadrupedFootSwitchInterface
{

   public QuadrupedSettableFootSwitch(ContactablePlaneBody foot, double totalRobotWeight, int totalNumberOfFeet, YoRegistry registry)
   {
      super(foot, totalRobotWeight, totalNumberOfFeet, registry);
   }

   @Override
   public void setFootContactState(boolean hasFootHitGround)
   {
      super.setFootContactState(hasFootHitGround);
   }

   @Override
   public void trustFootSwitchInSwing(boolean trustFootSwitch)
   {

   }

   @Override
   public void trustFootSwitchInSupport(boolean trustFootSwitch)
   {

   }
}
