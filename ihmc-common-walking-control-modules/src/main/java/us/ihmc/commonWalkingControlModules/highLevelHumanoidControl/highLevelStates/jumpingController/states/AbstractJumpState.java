package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController.states;

import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class AbstractJumpState extends FinishableState<JumpStateEnum>
{
   public AbstractJumpState(JumpStateEnum stateEnum)
   {
      super(stateEnum);
   }
   
   @Override
   public void doTransitionIntoAction()
   {
      
   }
}
