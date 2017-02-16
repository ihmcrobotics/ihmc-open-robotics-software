package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class RigidBodyUserControlState extends RigidBodyControlState
{

   public RigidBodyUserControlState(String bodyName, DoubleYoVariable yoTime)
   {
      super(RigidBodyControlMode.USER, bodyName, yoTime);
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void doAction()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void doTransitionIntoAction()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public void doTransitionOutOfAction()
   {
      // TODO Auto-generated method stub

   }

   public void setWeight(double userModeWeight)
   {
      // TODO Auto-generated method stub

   }

   @Override
   public boolean isEmpty()
   {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      // TODO Auto-generated method stub
      return 0;
   }

}
