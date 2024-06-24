package us.ihmc.behaviors.activeMapping.ContinuousHikingStatesAndTransitions;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.log.LogTools;
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
      if (!continuousHikingParameters.getEnableContinuousWalking() || !commandMessage.get().getEnableContinuousWalking())
      {
         LogTools.info(("STOPPING CONTINUOUS HIKING $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$44"));
         return true;
      }

      return false;
   }

}
