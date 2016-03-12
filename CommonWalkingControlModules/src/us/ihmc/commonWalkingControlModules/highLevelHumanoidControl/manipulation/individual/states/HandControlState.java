package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.HandControlMode;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.lowLevelControl.LowLevelOneDoFJointDesiredDataHolderInterface;
import us.ihmc.robotics.stateMachines.State;

public abstract class HandControlState extends State<HandControlMode>
{
   public HandControlState(HandControlMode stateEnum)
   {
      super(stateEnum);
   }

   public abstract InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();

   public abstract LowLevelOneDoFJointDesiredDataHolderInterface getLowLevelJointDesiredData();
}
