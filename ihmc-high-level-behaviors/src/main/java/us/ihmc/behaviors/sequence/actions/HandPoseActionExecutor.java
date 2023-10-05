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
import us.ihmc.behaviors.tools.HandWrenchCalculator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;

public class HandPoseActionExecutor implements BehaviorActionExecutor
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final HandPoseActionState state;
   private final HandPoseActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final HandWrenchCalculator handWrenchCalculator;
   private final HandPoseJointAnglesStatusMessage handPoseJointAnglesStatus = new HandPoseJointAnglesStatusMessage();
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();
   private final IKRootCalculator rootCalculator;

   public HandPoseActionExecutor(ROS2ControllerHelper ros2ControllerHelper,
                                 ReferenceFrameLibrary referenceFrameLibrary,
                                 DRCRobotModel robotModel,
                                 ROS2SyncedRobotModel syncedRobot,
                                 HandWrenchCalculator handWrenchCalculator)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.handWrenchCalculator = handWrenchCalculator;

      state = new HandPoseActionState(referenceFrameLibrary);
      definition = state.getDefinition();

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
      }
      rootCalculator = new IKRootCalculator(ros2ControllerHelper, syncedRobot.getFullRobotModel(), referenceFrameLibrary);
   }

   @Override
   public void update(int nextExecutionIndex, boolean concurrentActionIsNextForExecution)
   {
      boolean isNextForExecution = concurrentActionIsNextForExecution || state.getActionIndex() == nextExecutionIndex;

      if (isNextForExecution)
      {
         ArmIKSolver armIKSolver = armIKSolvers.get(definition.getSide());
         armIKSolver.copySourceToWork();
         rootCalculator.getKinematicsInfo();
         rootCalculator.computeRoot();
         ReferenceFrame chestFrame = rootCalculator.getRoot();
         armIKSolver.update(chestFrame, state.getPalmFrame().getReferenceFrame());
         armIKSolver.solve();

         // Send the solution back to the UI so the user knows what's gonna happen with the arm.
         handPoseJointAnglesStatus.getActionInformation().setActionIndex(state.getActionIndex());
         handPoseJointAnglesStatus.setRobotSide(definition.getSide().toByte());
         handPoseJointAnglesStatus.setSolutionQuality(armIKSolver.getQuality());
         for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
         {
            handPoseJointAnglesStatus.getJointAngles()[i] = armIKSolver.getSolutionOneDoFJoints()[i].getQ();
         }
         if (definition.getSide() == RobotSide.LEFT)
            ros2ControllerHelper.publish(BehaviorActionSequence.LEFT_HAND_POSE_JOINT_ANGLES_STATUS, handPoseJointAnglesStatus);
         else
            ros2ControllerHelper.publish(BehaviorActionSequence.RIGHT_HAND_POSE_JOINT_ANGLES_STATUS, handPoseJointAnglesStatus);
      }
   }

   @Override
   public void triggerActionExecution()
   {
      ArmIKSolver armIKSolver = armIKSolvers.get(definition.getSide());

      OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
      double[] jointAngles = new double[solutionOneDoFJoints.length];
      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = solutionOneDoFJoints[i].getQ();
      }

      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(definition.getSide(),
                                                                                                  definition.getTrajectoryDuration(),
                                                                                                  jointAngles);
      armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
      if (definition.getJointSpaceControl())
      {
         ros2ControllerHelper.publishToController(armTrajectoryMessage);
      }
      else
      {
         FramePose3D frameHand = new FramePose3D(state.getPalmFrame().getReferenceFrame());
         frameHand.changeFrame(ReferenceFrame.getWorldFrame());
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(definition.getSide(),
                                                                                                        definition.getTrajectoryDuration(),
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

         HandHybridJointspaceTaskspaceTrajectoryMessage hybridHandMessage
               = HumanoidMessageTools.createHandHybridJointspaceTaskspaceTrajectoryMessage(definition.getSide(),
                                                                                           handTrajectoryMessage.getSe3Trajectory(),
                                                                                           armTrajectoryMessage.getJointspaceTrajectory());
         ros2ControllerHelper.publishToController(hybridHandMessage);
      }

      executionTimer.reset();

      desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
      syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(definition.getSide()));
      startPositionDistanceToGoal = syncedHandControlPose.getTranslation().differenceNorm(desiredHandControlPose.getTranslation());
      startOrientationDistanceToGoal = syncedHandControlPose.getRotation().distance(desiredHandControlPose.getRotation(), true);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
      syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(definition.getSide()));

      boolean wasExecuting = isExecuting;
      // Left hand broke on Nadia and not in the robot model?
      isExecuting = !completionCalculator.isComplete(desiredHandControlPose,
                                                     syncedHandControlPose,
                                                     POSITION_TOLERANCE, ORIENTATION_TOLERANCE,
                                                     definition.getTrajectoryDuration(),
                                                     executionTimer,
                                                     BehaviorActionCompletionComponent.TRANSLATION,
                                                     BehaviorActionCompletionComponent.ORIENTATION);

      executionStatusMessage.setActionIndex(state.getActionIndex());
      executionStatusMessage.setNominalExecutionDuration(definition.getTrajectoryDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      executionStatusMessage.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
      executionStatusMessage.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
      executionStatusMessage.setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
      executionStatusMessage.setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
      executionStatusMessage.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      executionStatusMessage.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
      executionStatusMessage.setHandWrenchMagnitudeLinear(handWrenchCalculator.getLinearWrenchMagnitude(definition.getSide(), true));
      if (!isExecuting && wasExecuting && !definition.getJointSpaceControl() && !definition.getHoldPoseInWorldLater())
      {
         disengageHoldPoseInWorld();
      }
   }

   private void disengageHoldPoseInWorld()
   {
      ArmIKSolver armIKSolver = armIKSolvers.get(definition.getSide());

      OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
      double[] jointAngles = new double[solutionOneDoFJoints.length];
      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = solutionOneDoFJoints[i].getQ();
      }

      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(definition.getSide(),
                                                                                                  definition.getTrajectoryDuration(),
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
   public boolean isExecuting()
   {
      return isExecuting;
   }

   @Override
   public HandPoseActionState getState()
   {
      return state;
   }

   @Override
   public HandPoseActionDefinition getDefinition()
   {
      return definition;
   }
}
