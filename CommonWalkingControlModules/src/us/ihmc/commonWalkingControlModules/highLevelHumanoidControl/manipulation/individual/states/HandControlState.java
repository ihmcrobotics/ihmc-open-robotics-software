package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.robotics.stateMachines.State;

public abstract class HandControlState extends State<HandControlMode>
{
   public HandControlState(HandControlMode stateEnum)
   {
      super(stateEnum);
   }

   public abstract InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   public abstract LowLevelOneDoFJointDesiredDataHolderReadOnly getLowLevelJointDesiredData();
}
