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

public class HandPoseAction extends HandPoseActionDescription implements BehaviorAction
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private int actionIndex;
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

   public HandPoseAction(ROS2ControllerHelper ros2ControllerHelper,
                         ReferenceFrameLibrary referenceFrameLibrary,
                         DRCRobotModel robotModel,
                         ROS2SyncedRobotModel syncedRobot,
                         HandWrenchCalculator handWrenchCalculator)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.syncedRobot = syncedRobot;
      this.handWrenchCalculator = handWrenchCalculator;

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
      }
      rootCalculator = new IKRootCalculator(ros2ControllerHelper, syncedRobot.getFullRobotModel(), referenceFrameLibrary);
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex, boolean concurrencyWithPreviousIndex, int indexShiftConcurrentAction)
   {
      update(referenceFrameLibrary);

      this.actionIndex = actionIndex;

      // while the first action is being executed and the corresponding IK solution is computed, also do that for the following concurrent actions
      if (concurrencyWithPreviousIndex && actionIndex == (nextExecutionIndex + indexShiftConcurrentAction) ||
          (getExecuteWithNextAction() && actionIndex == nextExecutionIndex))
      {
         ArmIKSolver armIKSolver = armIKSolvers.get(getSide());
         armIKSolver.copyActualToWork();
         rootCalculator.getKinematicsInfo();
         rootCalculator.computeRoot();
         armIKSolver.setChestExternally(rootCalculator.getRoot());
         computeAndPublishIK(armIKSolver);
      }
      else if (actionIndex == nextExecutionIndex)
      {
         ArmIKSolver armIKSolver = armIKSolvers.get(getSide());
         armIKSolver.setChestExternally(null);
         armIKSolver.copyActualToWork();
         computeAndPublishIK(armIKSolver);
      }
   }

   private void computeAndPublishIK(ArmIKSolver armIKSolver)
   {
      armIKSolver.update(getConditionalReferenceFrame().get());
      armIKSolver.solve();

      // Send the solution back to the UI so the user knows what's gonna happen with the arm.
      handPoseJointAnglesStatus.getActionInformation().setActionIndex(actionIndex);
      handPoseJointAnglesStatus.setRobotSide(getSide().toByte());
      handPoseJointAnglesStatus.setSolutionQuality(armIKSolver.getQuality());
      for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
      {
         handPoseJointAnglesStatus.getJointAngles()[i] = armIKSolver.getSolutionOneDoFJoints()[i].getQ();
      }
      if (getSide() == RobotSide.LEFT)
         ros2ControllerHelper.publish(BehaviorActionSequence.LEFT_HAND_POSE_JOINT_ANGLES_STATUS, handPoseJointAnglesStatus);
      else
         ros2ControllerHelper.publish(BehaviorActionSequence.RIGHT_HAND_POSE_JOINT_ANGLES_STATUS, handPoseJointAnglesStatus);
   }

   @Override
   public void triggerActionExecution()
   {
      ArmIKSolver armIKSolver = armIKSolvers.get(getSide());

      OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
      double[] jointAngles = new double[solutionOneDoFJoints.length];
      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = solutionOneDoFJoints[i].getQ();
      }

      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getSide(), getTrajectoryDuration(), jointAngles);
      armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
      if (getJointSpaceControl())
      {
         ros2ControllerHelper.publishToController(armTrajectoryMessage);
      }
      else
      {
         FramePose3D frameHand = new FramePose3D(getConditionalReferenceFrame().get());
         frameHand.changeFrame(ReferenceFrame.getWorldFrame());
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(getSide(),
                                                                                                        getTrajectoryDuration(),
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

         HandHybridJointspaceTaskspaceTrajectoryMessage hybridHandMessage = HumanoidMessageTools.createHandHybridJointspaceTaskspaceTrajectoryMessage(getSide(),
                                                                                                                                                      handTrajectoryMessage.getSe3Trajectory(),
                                                                                                                                                      armTrajectoryMessage.getJointspaceTrajectory());
         ros2ControllerHelper.publishToController(hybridHandMessage);
      }

      executionTimer.reset();

      desiredHandControlPose.setFromReferenceFrame(getConditionalReferenceFrame().get());
      syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getSide()));
      startPositionDistanceToGoal = syncedHandControlPose.getTranslation().differenceNorm(desiredHandControlPose.getTranslation());
      startOrientationDistanceToGoal = syncedHandControlPose.getRotation().distance(desiredHandControlPose.getRotation(), true);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      desiredHandControlPose.setFromReferenceFrame(getConditionalReferenceFrame().get());
      syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getSide()));

      boolean wasExecuting = isExecuting;
      // Left hand broke on Nadia and not in the robot model?
      isExecuting = !completionCalculator.isComplete(desiredHandControlPose,
                                                     syncedHandControlPose,
                                                     POSITION_TOLERANCE, ORIENTATION_TOLERANCE,
                                                     getTrajectoryDuration(),
                                                     executionTimer,
                                                     BehaviorActionCompletionComponent.TRANSLATION,
                                                     BehaviorActionCompletionComponent.ORIENTATION);

      executionStatusMessage.setActionIndex(actionIndex);
      executionStatusMessage.setNominalExecutionDuration(getTrajectoryDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      executionStatusMessage.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
      executionStatusMessage.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
      executionStatusMessage.setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
      executionStatusMessage.setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
      executionStatusMessage.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      executionStatusMessage.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
      executionStatusMessage.setHandWrenchMagnitudeLinear(handWrenchCalculator.getLinearWrenchMagnitude(getSide(), true));
      if (!isExecuting && wasExecuting && !getJointSpaceControl() && !getHoldPoseInWorldLater())
      {
         disengageHoldPoseInWorld();
      }
   }

   private void disengageHoldPoseInWorld()
   {
      ArmIKSolver armIKSolver = armIKSolvers.get(getSide());

      OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
      double[] jointAngles = new double[solutionOneDoFJoints.length];
      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = solutionOneDoFJoints[i].getQ();
      }

      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getSide(), getTrajectoryDuration(), jointAngles);
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
}
