package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class HandControlState extends FinishableState<HandControlMode>
{
   public HandControlState(HandControlMode stateEnum)
   {
      super(stateEnum);
   }

   public boolean isAbortRequested()
   {
      return false;
   }

   public abstract InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();
}
