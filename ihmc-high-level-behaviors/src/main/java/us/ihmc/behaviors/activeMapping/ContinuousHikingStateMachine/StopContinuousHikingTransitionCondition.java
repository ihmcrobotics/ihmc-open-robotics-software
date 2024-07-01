package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

import java.util.concurrent.atomic.AtomicReference;

public class StopContinuousHikingTransitionCondition implements StateTransitionCondition
{
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;
   private final ContinuousHikingParameters continuousHikingParameters;

   /**
    * This transition is used in the {@link us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask} to determine whether the Continuous Hiking state
    * machine should be stopped. We want to be able to stop the state machine from whatever state we are in.
    */
   public StopContinuousHikingTransitionCondition(AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                                                  ContinuousHikingParameters continuousHikingParameters)
   {
      this.commandMessage = commandMessage;
      this.continuousHikingParameters = continuousHikingParameters;
   }

   @Override
   public boolean testCondition(double timeInCurrentState)
   {
      return !continuousHikingParameters.getEnableContinuousHiking() || !commandMessage.get().getEnableContinuousWalking();
   }
}
