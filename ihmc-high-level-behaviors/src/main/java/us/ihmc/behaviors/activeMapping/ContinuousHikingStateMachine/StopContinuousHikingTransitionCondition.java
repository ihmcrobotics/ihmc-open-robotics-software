package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

import java.util.concurrent.atomic.AtomicReference;

public class StopContinuousHikingTransitionCondition implements StateTransitionCondition
{
   private final ContinuousHikingParameters continuousHikingParameters;
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;

   public StopContinuousHikingTransitionCondition(ContinuousHikingParameters continuousHikingParameters,
                                                  AtomicReference<ContinuousWalkingCommandMessage> commandMessage)
   {
      this.continuousHikingParameters = continuousHikingParameters;
      this.commandMessage = commandMessage;
   }

   @Override
   public boolean testCondition(double timeInCurrentState)
   {
      return !continuousHikingParameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking();
   }
}
