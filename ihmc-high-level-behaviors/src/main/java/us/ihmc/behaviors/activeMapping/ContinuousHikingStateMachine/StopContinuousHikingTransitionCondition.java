package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

import java.util.concurrent.atomic.AtomicReference;

public class StopContinuousHikingTransitionCondition implements StateTransitionCondition
{
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;
   private final ContinuousHikingParameters continuousHikingParameters;

   public StopContinuousHikingTransitionCondition(AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                                                  ContinuousHikingParameters continuousHikingParameters)
   {
      this.commandMessage = commandMessage;
      this.continuousHikingParameters = continuousHikingParameters;
   }

   @Override
   public boolean testCondition(double timeInCurrentState)
   {
      return !continuousHikingParameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking();
   }
}
