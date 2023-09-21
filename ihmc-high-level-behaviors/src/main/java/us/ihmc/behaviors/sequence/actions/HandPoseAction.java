package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.behaviors.tools.HandWrenchCalculator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;

public class HandPoseAction extends HandPoseActionData implements BehaviorAction
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
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

   public HandPoseAction(ROS2ControllerHelper ros2ControllerHelper,
                         ReferenceFrameLibrary referenceFrameLibrary,
                         DRCRobotModel robotModel,
                         ROS2SyncedRobotModel syncedRobot,
                         HandWrenchCalculator handWrenchCalculator)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.handWrenchCalculator = handWrenchCalculator;
      setReferenceFrameLibrary(referenceFrameLibrary);

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
      }
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex, boolean concurrencyWithPreviousIndex, int indexShiftConcurrentAction)
   {
      update();

      this.actionIndex = actionIndex;

      // if this action has to be executed with the previous one, it means it belongs to a group of concurrent actions
      if (concurrencyWithPreviousIndex)
      {
         // while the first action is being executed and the corresponding IK solution is computed, also do that for the following concurrent actions
         if (actionIndex == (nextExecutionIndex + indexShiftConcurrentAction))
         {
            computeAndPublishIKSolution();
         }
      }
      if (actionIndex == nextExecutionIndex)
      {
         computeAndPublishIKSolution();
      }
   }

   private void computeAndPublishIKSolution()
   {
      ArmIKSolver armIKSolver = armIKSolvers.get(getSide());
      armIKSolver.copyActualToWork();
      armIKSolver.update(getPalmFrame());
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
      if (getJointSpaceControl())
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
      else
      {
         FramePose3D frameHand = new FramePose3D(getPalmFrame());
         frameHand.changeFrame(ReferenceFrame.getWorldFrame());
         HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(getSide(),
                                                                                                        getTrajectoryDuration(),
                                                                                                        frameHand.getPosition(),
                                                                                                        frameHand.getOrientation(),
                                                                                                        ReferenceFrame.getWorldFrame());
         handTrajectoryMessage.setForceExecution(true);
         ros2ControllerHelper.publishToController(handTrajectoryMessage);
      }

      executionTimer.reset();
      desiredHandControlPose.setFromReferenceFrame(getPalmFrame());
      syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getSide()));
      startPositionDistanceToGoal = syncedHandControlPose.getTranslation().differenceNorm(desiredHandControlPose.getTranslation());
      startOrientationDistanceToGoal = syncedHandControlPose.getRotation().distance(desiredHandControlPose.getRotation(), true);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      desiredHandControlPose.setFromReferenceFrame(getPalmFrame());
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
      FramePose3D frameHand = new FramePose3D(getPalmFrame());
      frameHand.changeFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());
      HandTrajectoryMessage message = HumanoidMessageTools.createHandTrajectoryMessage(getSide(),
                                                                                       getTrajectoryDuration(),
                                                                                       frameHand.getPosition(),
                                                                                       frameHand.getOrientation(),
                                                                                       syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());
      message.setForceExecution(true);
      ros2ControllerHelper.publishToController(message);
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
