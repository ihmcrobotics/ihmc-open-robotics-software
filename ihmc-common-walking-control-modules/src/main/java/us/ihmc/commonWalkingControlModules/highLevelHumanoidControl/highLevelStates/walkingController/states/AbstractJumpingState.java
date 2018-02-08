package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class AbstractJumpingState extends FinishableState<JumpStateEnum>
{

   public AbstractJumpingState(JumpStateEnum stateEnum)
   {
      super(stateEnum);
   }
}
