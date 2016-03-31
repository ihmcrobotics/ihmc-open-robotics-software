package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.stateTransitionConditions;

import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController.FlamingoSingleSupportState;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;

public class StopFlamingoCondition implements StateTransitionCondition
{
   private final FlamingoSingleSupportState singleSupportState;
   private final WalkingMessageHandler walkingMessageHandler;

   public StopFlamingoCondition(FlamingoSingleSupportState singleSupportState, WalkingMessageHandler walkingMessageHandler)
   {
      this.singleSupportState = singleSupportState;
      this.walkingMessageHandler = walkingMessageHandler;
   }

   @Override
   public boolean checkCondition()
   {
      if (!singleSupportState.isDone())
         return false;

      return !walkingMessageHandler.hasFootTrajectoryForFlamingoStance(singleSupportState.getSwingSide());
   }
}