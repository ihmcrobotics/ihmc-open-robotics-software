package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class HeadControlState extends FinishableState<HeadControlMode>
{
   public HeadControlState(HeadControlMode stateEnum)
   {
      super(stateEnum);
   }

   public abstract void setWeight(double weight);

   public abstract InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   @Override
   public boolean isDone()
   {
      return true;
   }
}
