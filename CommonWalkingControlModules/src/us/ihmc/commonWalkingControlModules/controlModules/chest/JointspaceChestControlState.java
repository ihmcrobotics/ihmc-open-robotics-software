package us.ihmc.commonWalkingControlModules.controlModules.chest;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SpineTrajectoryCommand;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class JointspaceChestControlState extends ChestControlState
{

   public JointspaceChestControlState()
   {
      super(ChestControlMode.JOINT_SPACE);
   }

   public void setWeight(double weight)
   {
      // TODO Auto-generated method stub

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

   public boolean handleNeckTrajectoryCommand(SpineTrajectoryCommand command, double[] initialJointPositions)
   {
      throw new RuntimeException("Implement me.");
   }

   public double getJointDesiredPosition(OneDoFJoint oneDoFJoint)
   {
      // TODO Auto-generated method stub
      return 0;
   }

   public double getJointDesiredVelocity(OneDoFJoint oneDoFJoint)
   {
      // TODO Auto-generated method stub
      return 0;
   }

}
