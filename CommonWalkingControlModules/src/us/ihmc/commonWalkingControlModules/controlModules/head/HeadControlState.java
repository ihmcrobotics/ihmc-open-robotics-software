package us.ihmc.commonWalkingControlModules.controlModules.head;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.stateMachines.State;

public abstract class HeadControlState extends State<HeadControlMode>
{
   public HeadControlState(HeadControlMode stateEnum)
   {
      super(stateEnum);
   }

   public abstract void setWeight(double weight);
   
   public abstract InverseDynamicsCommand<?> getInverseDynamicsCommand();

   public abstract FeedbackControlCommand<?> getFeedbackControlCommand();
}
