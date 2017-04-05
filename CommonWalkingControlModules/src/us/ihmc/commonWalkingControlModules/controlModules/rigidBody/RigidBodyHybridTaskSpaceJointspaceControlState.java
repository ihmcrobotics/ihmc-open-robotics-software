package us.ihmc.commonWalkingControlModules.controlModules.rigidBody;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commons.PrintTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.JointspaceTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SE3TrajectoryControllerCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SO3TrajectoryControllerCommand;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

public class RigidBodyHybridTaskSpaceJointspaceControlState extends RigidBodyControlState
{
   private final InverseDynamicsCommandList inverseDynamicsCommandList = new InverseDynamicsCommandList();
   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();

   private final RigidBodyJointspaceControlState jointspaceControlState;
   private final RigidBodyTaskspaceControlState taskspaceControlState;
   private String bodyName;

   public RigidBodyHybridTaskSpaceJointspaceControlState(String bodyName, RigidBodyJointspaceControlState jointSpaceState, RigidBodyTaskspaceControlState taskspaceControlState, DoubleYoVariable yoTime, YoVariableRegistry parentRegistry)
   {
      super(RigidBodyControlMode.HYBRID, bodyName, yoTime, parentRegistry);
      this.bodyName = bodyName;
      this.jointspaceControlState = jointSpaceState;
      this.taskspaceControlState = taskspaceControlState;
   }

   @Override
   public void doAction()
   {
      jointspaceControlState.doAction();
      taskspaceControlState.doAction();
   }

   public boolean handleTrajectoryCommand(SE3TrajectoryControllerCommand<?,?> se3TaskspaceCommand, JointspaceTrajectoryCommand<?,?> jointspaceCommand, double[] initialJointPositions)
   {
      if (!taskspaceControlState.handlePoseTrajectoryCommand(se3TaskspaceCommand))
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid pose trajectory command.");
         taskspaceControlState.clear();
         return false;
      }

      if (!jointspaceControlState.handleTrajectoryCommand(jointspaceCommand, initialJointPositions))
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid jointspace trajectory command.");
         return false;
      }
      return true;

   }

   public boolean handleTrajectoryCommand(SO3TrajectoryControllerCommand<?,?> so3TaskspaceCommand, JointspaceTrajectoryCommand<?,?> jointspaceCommand, double[] initialJointPositions)
   {
      if (!taskspaceControlState.handleOrientationTrajectoryCommand(so3TaskspaceCommand))
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid pose trajectory command.");
         taskspaceControlState.clear();
         return false;
      }

      if (!jointspaceControlState.handleTrajectoryCommand(jointspaceCommand, initialJointPositions))
      {
         PrintTools.warn(getClass().getSimpleName() + " for " + bodyName + " recieved invalid jointspace trajectory command.");
         return false;
      }
      return true;
   }


   @Override
   public double getLastTrajectoryPointTime()
   {
      return Math.max(jointspaceControlState.getLastTrajectoryPointTime(), taskspaceControlState.getLastTrajectoryPointTime());
   }

   @Override
   public boolean isEmpty()
   {
      return jointspaceControlState.isEmpty() && taskspaceControlState.isEmpty();
   };

   @Override
   public void doTransitionIntoAction()
   {
      jointspaceControlState.doTransitionIntoAction();
      taskspaceControlState.doTransitionIntoAction();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      jointspaceControlState.doTransitionOutOfAction();
      taskspaceControlState.doTransitionOutOfAction();
   }

   @Override
   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      inverseDynamicsCommandList.clear();
      inverseDynamicsCommandList.addCommand(jointspaceControlState.getInverseDynamicsCommand());
      inverseDynamicsCommandList.addCommand(taskspaceControlState.getInverseDynamicsCommand());
      return inverseDynamicsCommandList;
   }

   @Override
   public FeedbackControlCommand<?> getFeedbackControlCommand()
   {
      feedbackControlCommandList.clear();
      feedbackControlCommandList.addCommand(jointspaceControlState.getFeedbackControlCommand());
      feedbackControlCommandList.addCommand(taskspaceControlState.getFeedbackControlCommand());
      return feedbackControlCommandList;
   }
}
