package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSE3HybridJointSpaceTaskSpaceTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class RigidBodyHybridTaskSpaceJointspaceControlState extends RigidBodyControlState
{
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private final RigidBodyJointspaceControlState jointSpaceState;
   private final RigidBodyTaskspaceControlState taskspaceControlState;

   public RigidBodyHybridTaskSpaceJointspaceControlState(String bodyName, RigidBodyJointspaceControlState jointSpaceState, RigidBodyTaskspaceControlState taskspaceControlState, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.HYBRID, bodyName, yoTime, parentRegistry);
      this.jointSpaceState = jointSpaceState;
      this.taskspaceControlState = taskspaceControlState;
   }

   @Override
   public void doAction()
   {
      jointSpaceState.doAction();
      taskspaceControlState.doAction();
   }

   public boolean handleTrajectoryCommand(AbstractSE3HybridJointSpaceTaskSpaceTrajectoryMessage<?, ?, ?> command, double[] initialJointPositions)
   {
      return false;
   }
   
   public boolean handleTrajectoryCommand(AbstractSO3HybridJointSpaceTaskSpaceTrajectoryMessage<?, ?, ?> command, double[] initialJointPositions)
   {
      return false;
   }


   @Override
   public double getLastTrajectoryPointTime()
   {
      return Math.max(jointSpaceState.getLastTrajectoryPointTime(), taskspaceControlState.getLastTrajectoryPointTime());
   }

   @Override
   public boolean isEmpty()
   {
      return jointSpaceState.isEmpty() && taskspaceControlState.isEmpty();
   };

   @Override
   public void doTransitionIntoAction()
   {
      jointSpaceState.doTransitionIntoAction();
      taskspaceControlState.doTransitionIntoAction();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      jointSpaceState.doTransitionOutOfAction();
      taskspaceControlState.doTransitionOutOfAction();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(jointSpaceState.getInverseDynamicsCommand());
      inverseDynamicsCommandList.addCommand(taskspaceControlState.getInverseDynamicsCommand());
      return inverseDynamicsCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      feedbackControlCommandList.clear();
      feedbackControlCommandList.addCommand(jointSpaceState.getFeedbackControlCommand());
      feedbackControlCommandList.addCommand(taskspaceControlState.getFeedbackControlCommand());
      return feedbackControlCommandList;
   }
}
