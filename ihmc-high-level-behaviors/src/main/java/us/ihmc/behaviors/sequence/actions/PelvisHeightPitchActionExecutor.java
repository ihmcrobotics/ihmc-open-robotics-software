package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.behaviors.sequence.BehaviorActionExecutor;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.Timer;

public class PelvisHeightPitchActionExecutor extends BehaviorActionExecutor
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final PelvisHeightPitchActionState state;
   private final PelvisHeightPitchActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final Timer executionTimer = new Timer();
   private final FramePose3D desiredPelvisPose = new FramePose3D();
   private final FramePose3D syncedPelvisPose = new FramePose3D();
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();

   public PelvisHeightPitchActionExecutor(BehaviorActionSequence sequence,
                                          ROS2ControllerHelper ros2ControllerHelper,
                                          ReferenceFrameLibrary referenceFrameLibrary,
                                          ROS2SyncedRobotModel syncedRobot)
   {
      super(sequence);

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;

      state = new PelvisHeightPitchActionState(referenceFrameLibrary);
      definition = state.getDefinition();
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerActionExecution()
   {
      if (state.getPelvisFrame().isChildOfWorld())
      {
         FramePose3D framePose = new FramePose3D(state.getPelvisFrame().getReferenceFrame());
         FramePose3D syncedPose = new FramePose3D(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
         framePose.getRotation().setYawPitchRoll(syncedPose.getYaw(), framePose.getPitch(), syncedPose.getRoll());
         framePose.changeFrame(ReferenceFrame.getWorldFrame());
         syncedPose.changeFrame(ReferenceFrame.getWorldFrame());

         PelvisTrajectoryMessage message = new PelvisTrajectoryMessage();
         message.getSe3Trajectory()
                .set(HumanoidMessageTools.createSE3TrajectoryMessage(definition.getTrajectoryDuration(),
                                                                     framePose.getPosition(),
                                                                     framePose.getOrientation(),
                                                                     ReferenceFrame.getWorldFrame()));
         long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);
         message.getSe3Trajectory().getLinearSelectionMatrix().setXSelected(false);
         message.getSe3Trajectory().getLinearSelectionMatrix().setYSelected(false);
         message.getSe3Trajectory().getLinearSelectionMatrix().setZSelected(true);
         ros2ControllerHelper.publishToController(message);
         executionTimer.reset();

         desiredPelvisPose.setFromReferenceFrame(state.getPelvisFrame().getReferenceFrame());
         syncedPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
         desiredPelvisPose.getTranslation().set(syncedPelvisPose.getTranslationX(), syncedPelvisPose.getTranslationY(), desiredPelvisPose.getTranslationZ());
         desiredPelvisPose.getRotation().setYawPitchRoll(syncedPelvisPose.getYaw(), desiredPelvisPose.getPitch(), syncedPelvisPose.getRoll());
         startPositionDistanceToGoal = syncedPelvisPose.getTranslation().differenceNorm(desiredPelvisPose.getTranslation());
         startOrientationDistanceToGoal = syncedPelvisPose.getRotation().distance(desiredPelvisPose.getRotation(), true);
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (state.getPelvisFrame().isChildOfWorld())
      {
         desiredPelvisPose.setFromReferenceFrame(state.getPelvisFrame().getReferenceFrame());
         syncedPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
         desiredPelvisPose.getTranslation().set(syncedPelvisPose.getTranslationX(), syncedPelvisPose.getTranslationY(), desiredPelvisPose.getTranslationZ());
         desiredPelvisPose.getRotation().setYawPitchRoll(syncedPelvisPose.getYaw(), desiredPelvisPose.getPitch(), syncedPelvisPose.getRoll());

         state.setIsExecuting(!completionCalculator.isComplete(desiredPelvisPose,
                                                               syncedPelvisPose,
                                                               POSITION_TOLERANCE,
                                                               Double.NaN,
                                                               definition.getTrajectoryDuration(),
                                                               executionTimer,
                                                               BehaviorActionCompletionComponent.TRANSLATION));

         executionStatusMessage.setActionIndex(state.getActionIndex());
         executionStatusMessage.setNominalExecutionDuration(definition.getTrajectoryDuration());
         executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
         executionStatusMessage.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
         executionStatusMessage.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
         executionStatusMessage.setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
         executionStatusMessage.setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
         executionStatusMessage.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
         executionStatusMessage.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
      }
   }

   @Override
   public ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return executionStatusMessage;
   }

   @Override
   public PelvisHeightPitchActionState getState()
   {
      return state;
   }

   @Override
   public PelvisHeightPitchActionDefinition getDefinition()
   {
      return definition;
   }
}
