package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class RigidBodyLoadBearingControlState extends RigidBodyControlState
{

   public RigidBodyLoadBearingControlState(String bodyName, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.LOAD_BEARING, bodyName, yoTime);
      parentRegistry.addChild(registry);
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

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      return null;
   }

   @Override
   public boolean isEmpty()
   {
      // this control mode does not support command queuing
      return false;
   }

   @Override
   public double getLastTrajectoryPointTime()
   {
      // this control mode does not support command queuing
      return 0.0;
   }

}
