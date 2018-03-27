package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class AbstractJumpingState extends FinishableState<JumpStateEnum>
{

   public AbstractJumpingState(JumpStateEnum stateEnum)
   {
      super(stateEnum);
   }
}
