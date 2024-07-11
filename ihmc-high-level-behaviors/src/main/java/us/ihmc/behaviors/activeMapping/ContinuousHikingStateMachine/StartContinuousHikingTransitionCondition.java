package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

import java.util.concurrent.atomic.AtomicReference;

public class StartContinuousHikingTransitionCondition implements StateTransitionCondition
{
   private final AtomicReference<ContinuousWalkingCommandMessage> commandMessage;
   private final ContinuousHikingParameters continuousHikingParameters;

   /**
    * This transition is used in the {@link us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask} to determine whether the Continuous Hiking state
    * machine should be started.
    */
   public StartContinuousHikingTransitionCondition(AtomicReference<ContinuousWalkingCommandMessage> commandMessage,
                                                   ContinuousHikingParameters continuousHikingParameters)
   {
      this.commandMessage = commandMessage;
      this.continuousHikingParameters = continuousHikingParameters;
   }

   @Override
   public boolean testCondition(double timeInCurrentState)
   {
      // Both conditions have to be true in order for this to work. The makes things a bit safer to use and can prevent accidentally starting things and having the robot walk
      return continuousHikingParameters.getEnableContinuousHiking() && commandMessage.get().getEnableContinuousWalking();
   }
}
