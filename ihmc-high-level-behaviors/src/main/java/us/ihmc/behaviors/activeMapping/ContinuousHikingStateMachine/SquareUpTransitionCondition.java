package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.ContinuousHikingCommandMessage;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

import java.util.concurrent.atomic.AtomicReference;

public class SquareUpTransitionCondition implements StateTransitionCondition
{
   private final AtomicReference<ContinuousHikingCommandMessage> commandMessage;

   public SquareUpTransitionCondition(AtomicReference<ContinuousHikingCommandMessage> commandMessage)
   {
      this.commandMessage = commandMessage;
   }

   @Override
   public boolean testCondition(double timeInCurrentState)
   {
      return commandMessage.get().getSquareUpToGoal() && !commandMessage.get().getEnableContinuousHiking();
   }
}
