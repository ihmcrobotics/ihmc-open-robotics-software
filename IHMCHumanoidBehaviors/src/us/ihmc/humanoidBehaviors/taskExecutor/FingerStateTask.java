package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.humanoidBehaviors.behaviors.primitives.HandDesiredConfigurationBehavior;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandDesiredConfigurationMessage;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;

public class FingerStateTask extends BehaviorTask
{
   private static final boolean DEBUG = false;

   private final HandDesiredConfigurationMessage fingerStatePacket;
   private final HandDesiredConfigurationBehavior fingerStateBehavior;

   public FingerStateTask(RobotSide robotSide, HandConfiguration fingerState, HandDesiredConfigurationBehavior fingerStateBehavior, DoubleYoVariable yoTime)
   {
      super(fingerStateBehavior, yoTime);
      this.fingerStatePacket = new HandDesiredConfigurationMessage(robotSide, fingerState);
      this.fingerStateBehavior = fingerStateBehavior;
   }

   @Override
   protected void setBehaviorInput()
   {
      fingerStateBehavior.setInput(fingerStatePacket);
   }
}
