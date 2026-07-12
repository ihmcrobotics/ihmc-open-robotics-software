package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.ContinuousHikingCommandMessage;
import us.ihmc.behaviors.activeMapping.ContinuousPlanningStateMachine;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

import java.util.concurrent.atomic.AtomicReference;

public class StartContinuousHikingTransitionCondition implements StateTransitionCondition
{
   private final AtomicReference<ContinuousHikingCommandMessage> commandMessage;

   /**
    * This transition is used in the {@link ContinuousPlanningStateMachine} to determine whether the Continuous Hiking state
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
