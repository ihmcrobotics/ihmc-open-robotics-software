package us.ihmc.behaviors.activeMapping;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

import java.util.concurrent.atomic.AtomicReference;

public class StopContinuousWalkingTransitionCondition implements StateTransitionCondition
{
   private final ContinuousHikingParameters continuousHikingParameters;
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;

   public StopContinuousWalkingTransitionCondition(ContinuousHikingParameters continuousHikingParameters,
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
