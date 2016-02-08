package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.HandDesiredConfigurationBehavior;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandDesiredConfigurationTask extends BehaviorTask
{
   private final HandDesiredConfigurationMessage ihmcMessage;
   private final HandDesiredConfigurationBehavior behavior;

   public HandDesiredConfigurationTask(RobotSide robotSide, HandConfiguration handDesiredConfiguration, HandDesiredConfigurationBehavior handDesiredConfigurationBehavior, DoubleYoVariable yoTime)
   {
      super(handDesiredConfigurationBehavior, yoTime);
      this.ihmcMessage = new HandDesiredConfigurationMessage(robotSide, handDesiredConfiguration);
      this.behavior = handDesiredConfigurationBehavior;
   }

   @Override
   protected void setBehaviorInput()
   {
      behavior.setInput(ihmcMessage);
   }
}
