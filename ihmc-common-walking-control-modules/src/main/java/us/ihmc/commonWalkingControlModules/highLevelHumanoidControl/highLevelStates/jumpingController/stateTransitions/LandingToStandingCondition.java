package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.stateTransitions;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states.LandingState;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateTransitionCondition;

public class LandingToStandingCondition implements StateTransitionCondition
{
   private final LandingState landingState;

   public LandingToStandingCondition(LandingState landingState)
   {
      this.landingState = landingState;
   }

   @Override
   public boolean checkCondition()
   {
      return landingState.isDone();
   }
}
