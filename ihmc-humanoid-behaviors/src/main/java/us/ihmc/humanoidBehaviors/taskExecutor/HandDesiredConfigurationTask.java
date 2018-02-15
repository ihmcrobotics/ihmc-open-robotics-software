package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.HandDesiredConfigurationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandDesiredConfigurationTask<E extends Enum<E>> extends BehaviorAction<E>
{
   private final HandDesiredConfigurationMessage ihmcMessage;
   private final HandDesiredConfigurationBehavior behavior;

   public HandDesiredConfigurationTask(RobotSide robotSide, HandConfiguration handDesiredConfiguration, HandDesiredConfigurationBehavior handDesiredConfigurationBehavior)
   {
      this(null, robotSide, handDesiredConfiguration, handDesiredConfigurationBehavior);
   }

   
   public HandDesiredConfigurationTask(E stateEnum,RobotSide robotSide, HandConfiguration handDesiredConfiguration, HandDesiredConfigurationBehavior handDesiredConfigurationBehavior)
   {
      super(stateEnum,handDesiredConfigurationBehavior);
      this.ihmcMessage = HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide, handDesiredConfiguration);
      this.behavior = handDesiredConfigurationBehavior;
   }

   @Override
   protected void setBehaviorInput()
   {
      behavior.setInput(ihmcMessage);
   }
}
