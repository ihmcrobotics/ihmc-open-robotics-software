package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.msg.dds.ContinuousHikingCommandMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

import java.util.concurrent.atomic.AtomicReference;

public class StartContinuousHikingTransitionCondition implements StateTransitionCondition
{
   private final AtomicReference<ContinuousHikingCommandMessage> commandMessage;

   /**
    * This transition is used in the {@link us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask} to determine whether the Continuous Hiking state
    * machine should be started.
    */
   public StartContinuousHikingTransitionCondition(AtomicReference<ContinuousHikingCommandMessage> commandMessage)
   {
      this.commandMessage = commandMessage;
   }

   @Override
   public boolean testCondition(double timeInCurrentState)
   {
      return commandMessage.get().getEnableContinuousHiking();
   }
}
