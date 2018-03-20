package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.HandDesiredConfigurationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandDesiredConfigurationTask extends BehaviorAction
{
   private final HandDesiredConfigurationMessage ihmcMessage;
   private final HandDesiredConfigurationBehavior behavior;

   public HandDesiredConfigurationTask(RobotSide robotSide, HandConfiguration handDesiredConfiguration,
                                       HandDesiredConfigurationBehavior handDesiredConfigurationBehavior)
   {
      super(handDesiredConfigurationBehavior);
      this.ihmcMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide, handDesiredConfiguration);
      this.behavior = handDesiredConfigurationBehavior;
   }

   @Override
   protected void setBehaviorInput()
   {
      behavior.setInput(ihmcMessage);
   }
}
