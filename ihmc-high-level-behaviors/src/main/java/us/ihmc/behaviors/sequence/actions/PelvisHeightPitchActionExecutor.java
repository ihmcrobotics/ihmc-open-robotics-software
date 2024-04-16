package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.StopAllTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.TaskspaceTrajectoryTrackingErrorCalculator;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class PelvisHeightPitchActionExecutor extends ActionNodeExecutor<PelvisHeightPitchActionState, PelvisHeightPitchActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;

   private final PelvisHeightPitchActionDefinition definition;
   private final PelvisHeightPitchActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FramePose3D desiredPelvisPose = new FramePose3D();
   private final FramePose3D syncedPelvisPose = new FramePose3D();
   private final TaskspaceTrajectoryTrackingErrorCalculator trackingCalculator = new TaskspaceTrajectoryTrackingErrorCalculator();
   private final transient StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();

   public PelvisHeightPitchActionExecutor(long id,
                                          CRDTInfo crdtInfo,
                                          WorkspaceResourceDirectory saveFileDirectory,
                                          ROS2ControllerHelper ros2ControllerHelper,
                                          ReferenceFrameLibrary referenceFrameLibrary,
                                          ROS2SyncedRobotModel syncedRobot)
   {
      super(new PelvisHeightPitchActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;

      definition = getDefinition();
   }

   @Override
   public void update()
   {
      super.update();

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      state.setCanExecute(state.getPelvisFrame().isChildOfWorld());
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

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

         trackingCalculator.reset();

         state.setNominalExecutionDuration(definition.getTrajectoryDuration());

         desiredPelvisPose.setFromReferenceFrame(state.getPelvisFrame().getReferenceFrame());
         syncedPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
         desiredPelvisPose.getTranslation().set(syncedPelvisPose.getTranslationX(), syncedPelvisPose.getTranslationY(), desiredPelvisPose.getTranslationZ());
         desiredPelvisPose.getRotation().setYawPitchRoll(syncedPelvisPose.getYaw(), desiredPelvisPose.getPitch(), syncedPelvisPose.getRoll());
         state.getCommandedTrajectory().setSingleSegmentTrajectory(syncedPelvisPose, desiredPelvisPose, getDefinition().getTrajectoryDuration());
      }
      else
      {
         state.getLogger().error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      trackingCalculator.computeExecutionTimings(state.getNominalExecutionDuration());
      state.setElapsedExecutionTime(trackingCalculator.getElapsedTime());

      if (trackingCalculator.getHitTimeLimit())
      {
         state.setIsExecuting(false);
         state.setFailed(true);
         state.getLogger().error("Task execution timed out. Publishing stop all trajectories message.");
         ros2ControllerHelper.publishToController(stopAllTrajectoryMessage);
         return;
      }

      if (state.getPelvisFrame().isChildOfWorld())
      {
         desiredPelvisPose.setFromReferenceFrame(state.getPelvisFrame().getReferenceFrame());
         syncedPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
         desiredPelvisPose.getTranslation().set(syncedPelvisPose.getTranslationX(), syncedPelvisPose.getTranslationY(), desiredPelvisPose.getTranslationZ());
         desiredPelvisPose.getRotation().setYawPitchRoll(syncedPelvisPose.getYaw(), desiredPelvisPose.getPitch(), syncedPelvisPose.getRoll());

         trackingCalculator.computePoseTrackingData(desiredPelvisPose, syncedPelvisPose);
         trackingCalculator.factorInR3Errors(POSITION_TOLERANCE);

         boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();

         if (meetsDesiredCompletionCriteria)
         {
            state.setIsExecuting(false);
         }

         state.getCurrentPose().getValue().set(syncedPelvisPose);
         state.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      }
   }
}
