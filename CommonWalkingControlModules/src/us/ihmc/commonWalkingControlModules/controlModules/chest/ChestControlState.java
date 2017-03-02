package us.ihmc.commonWalkingControlModules.controlModules.chest;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StopAllTrajectoryCommand;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;

public abstract class ChestControlState extends FinishableState<ChestControlMode>
{
   public ChestControlState(ChestControlMode stateEnum)
   {
      super(stateEnum);
   }

   public abstract void handleStopAllTrajectoryCommand(StopAllTrajectoryCommand command);

   public abstract InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   @Override
   public boolean isDone()
   {
      return true;
   }
}
