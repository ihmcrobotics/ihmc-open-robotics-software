package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.*;
import us.ihmc.behaviors.sequence.ros2.ROS2BehaviorActionSequence;
import us.ihmc.behaviors.tools.ROS2HandWrenchCalculator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;

public class HandPoseActionExecutor extends BehaviorActionExecutor<HandPoseActionState, HandPoseActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final HandPoseActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators;
   private final HandPoseJointAnglesStatusMessage handPoseJointAnglesStatus = new HandPoseJointAnglesStatusMessage();
   private final Timer executionTimer = new Timer();
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();
   private final IKRootCalculator rootCalculator;

   public HandPoseActionExecutor(long id,
                                 ROS2ControllerHelper ros2ControllerHelper,
                                 ReferenceFrameLibrary referenceFrameLibrary,
                                 DRCRobotModel robotModel,
                                 ROS2SyncedRobotModel syncedRobot,
                                 SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.handWrenchCalculators = handWrenchCalculators;

      state = new HandPoseActionState(id, referenceFrameLibrary);

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
      }
      rootCalculator = new IKRootCalculator(ros2ControllerHelper, syncedRobot.getFullRobotModel(), referenceFrameLibrary);
   }

   @Override
   public void update()
   {
      super.update();

      if (state.getPalmFrame().isChildOfWorld() && state.getIsNextForExecution())
      {
         ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());
         armIKSolver.copySourceToWork();
         rootCalculator.getKinematicsInfo();
         rootCalculator.computeRoot();
         ReferenceFrame chestFrame = rootCalculator.getRoot();
         armIKSolver.update(chestFrame, state.getPalmFrame().getReferenceFrame());
         armIKSolver.solve();

         // Send the solution back to the UI so the user knows what's gonna happen with the arm.
         handPoseJointAnglesStatus.getActionInformation().setActionIndex(state.getActionIndex());
         handPoseJointAnglesStatus.setRobotSide(getDefinition().getSide().toByte());
         handPoseJointAnglesStatus.setSolutionQuality(armIKSolver.getQuality());
         for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
         {
            handPoseJointAnglesStatus.getJointAngles()[i] = armIKSolver.getSolutionOneDoFJoints()[i].getQ();
         }
         if (getDefinition().getSide() == RobotSide.LEFT)
            ros2ControllerHelper.publish(ROS2BehaviorActionSequence.LEFT_HAND_POSE_JOINT_ANGLES_STATUS, handPoseJointAnglesStatus);
         else
            ros2ControllerHelper.publish(ROS2BehaviorActionSequence.RIGHT_HAND_POSE_JOINT_ANGLES_STATUS, handPoseJointAnglesStatus);
      }
   }

   @Override
   public void triggerActionExecution()
   {
      if (state.getPalmFrame().isChildOfWorld())
      {
         ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());

         OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
         double[] jointAngles = new double[solutionOneDoFJoints.length];
         for (int i = 0; i < jointAngles.length; i++)
         {
            jointAngles[i] = solutionOneDoFJoints[i].getQ();
         }

         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getDefinition().getSide(),
                                                                                                     getDefinition().getTrajectoryDuration(),
                                                                                                     jointAngles);
         armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
         if (getDefinition().getJointSpaceControl())
         {
            ros2ControllerHelper.publishToController(armTrajectoryMessage);
         }
         else
         {
            FramePose3D frameHand = new FramePose3D(state.getPalmFrame().getReferenceFrame());
            frameHand.changeFrame(ReferenceFrame.getWorldFrame());
            HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(getDefinition().getSide(),
                                                                                                           getDefinition().getTrajectoryDuration(),
                                                                                                           frameHand.getPosition(),
                                                                                                           frameHand.getOrientation(),
                                                                                                           ReferenceFrame.getWorldFrame());
            handTrajectoryMessage.getSe3Trajectory().getAngularWeightMatrix().setXWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getAngularWeightMatrix().setYWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getAngularWeightMatrix().setZWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getLinearWeightMatrix().setXWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getLinearWeightMatrix().setYWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getLinearWeightMatrix().setZWeight(50.0);
            handTrajectoryMessage.setForceExecution(true);

            HandHybridJointspaceTaskspaceTrajectoryMessage hybridHandMessage = HumanoidMessageTools.createHandHybridJointspaceTaskspaceTrajectoryMessage(
                  getDefinition().getSide(),
                  handTrajectoryMessage.getSe3Trajectory(),
                  armTrajectoryMessage.getJointspaceTrajectory());
            ros2ControllerHelper.publishToController(hybridHandMessage);
         }

         executionTimer.reset();

         desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));
         startPositionDistanceToGoal = syncedHandControlPose.getTranslation().differenceNorm(desiredHandControlPose.getTranslation());
         startOrientationDistanceToGoal = syncedHandControlPose.getRotation().distance(desiredHandControlPose.getRotation(), true);
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (state.getPalmFrame().isChildOfWorld())
      {
         desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));

         boolean wasExecuting = state.getIsExecuting();
         // Left hand broke on Nadia and not in the robot model?
         state.setIsExecuting(!completionCalculator.isComplete(desiredHandControlPose,
                                                               syncedHandControlPose,
                                                               POSITION_TOLERANCE,
                                                               ORIENTATION_TOLERANCE,
                                                               getDefinition().getTrajectoryDuration(),
                                                               executionTimer,
                                                               BehaviorActionCompletionComponent.TRANSLATION,
                                                               BehaviorActionCompletionComponent.ORIENTATION));

         executionStatusMessage.setActionIndex(state.getActionIndex());
         executionStatusMessage.setNominalExecutionDuration(getDefinition().getTrajectoryDuration());
         executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
         executionStatusMessage.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
         executionStatusMessage.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
         executionStatusMessage.setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
         executionStatusMessage.setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
         executionStatusMessage.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
         executionStatusMessage.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
         executionStatusMessage.setHandWrenchMagnitudeLinear(handWrenchCalculators.get(getDefinition().getSide()).getLinearWrenchMagnitude(true));
         if (!state.getIsExecuting() && wasExecuting && !getDefinition().getJointSpaceControl() && !getDefinition().getHoldPoseInWorldLater())
         {
            disengageHoldPoseInWorld();
         }
      }
   }

   private void disengageHoldPoseInWorld()
   {
      ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());

      OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
      double[] jointAngles = new double[solutionOneDoFJoints.length];
      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = solutionOneDoFJoints[i].getQ();
      }

      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getDefinition().getSide(),
                                                                                                  getDefinition().getTrajectoryDuration(),
                                                                                                  jointAngles);
      armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
      ros2ControllerHelper.publishToController(armTrajectoryMessage);
   }

   @Override
   public ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return executionStatusMessage;
   }

   @Override
   public HandPoseActionState getState()
   {
      return state;
   }
}
