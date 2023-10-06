package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage;
import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.*;
import us.ihmc.behaviors.tools.HandWrenchCalculator;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;

public class FootPoseActionExecutor extends FootPoseActionDefinition implements BehaviorActionExecutor
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private int actionIndex;
   private final FramePose3D desiredFootControlPose = new FramePose3D();
   private final FramePose3D syncedFootControlPose = new FramePose3D();
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();

   public FootPoseActionExecutor(ROS2ControllerHelper ros2ControllerHelper,
                                 ROS2SyncedRobotModel syncedRobot,
                                 ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      setReferenceFrameLibrary(referenceFrameLibrary);
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex, boolean concurrentActionIsNextForExecution)
   {
      update();

      this.actionIndex = actionIndex;
   }

   @Override
   public void triggerActionExecution()
   {
      FramePose3D frameFootPose = new FramePose3D(getFootFrame());
      frameFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage message = HumanoidMessageTools.createFootTrajectoryMessage(getSide(),
                                                      getTrajectoryDuration(), frameFootPose);
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

      ros2ControllerHelper.publishToController(message);
      executionTimer.reset();

      desiredFootControlPose.setFromReferenceFrame(getFootFrame());
      syncedFootControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getSoleFrame(getSide()));
      startPositionDistanceToGoal = syncedFootControlPose.getTranslation().differenceNorm(desiredFootControlPose.getTranslation());
      startOrientationDistanceToGoal = syncedFootControlPose.getRotation().distance(desiredFootControlPose.getRotation(), true);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      desiredFootControlPose.setFromReferenceFrame(getFootFrame());
      syncedFootControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getSoleFrame(getSide()));

      boolean wasExecuting = isExecuting;
      // Left hand broke on Nadia and not in the robot model?
      isExecuting = !completionCalculator.isComplete(desiredFootControlPose,
                                                     syncedFootControlPose,
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
