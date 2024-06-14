package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.FootTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.TaskspaceTrajectoryTrackingErrorCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FootPoseActionExecutor extends ActionNodeExecutor<FootPoseActionState, FootPoseActionDefinition>
{
   private final FootPoseActionState state;
   private final FootPoseActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FramePose3D desiredFootPose = new FramePose3D();
   private final FramePose3D syncedFootPose = new FramePose3D();
   private final TaskspaceTrajectoryTrackingErrorCalculator trackingCalculator = new TaskspaceTrajectoryTrackingErrorCalculator();

   public FootPoseActionExecutor(long id,
                                 CRDTInfo crdtInfo,
                                 WorkspaceResourceDirectory saveFileDirectory,
                                 ROS2ControllerHelper ros2ControllerHelper,
                                 ReferenceFrameLibrary referenceFrameLibrary,
                                 ROS2SyncedRobotModel syncedRobot)
   {
      super(new FootPoseActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
   }

   @Override
   public void update()
   {
      super.update();

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      state.setCanExecute(state.getFootFrame().isChildOfWorld());
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      if (state.getFootFrame().isChildOfWorld())
      {
         FramePose3D desiredControlFramePose = new FramePose3D(state.getFootFrame().getReferenceFrame());
         desiredControlFramePose.changeFrame(ReferenceFrame.getWorldFrame());

         FootTrajectoryMessage message = new FootTrajectoryMessage();
         message.setRobotSide(getDefinition().getSide().toByte());
         message.getSe3Trajectory()
                .set(HumanoidMessageTools.createSE3TrajectoryMessage(getDefinition().getTrajectoryDuration(),
                                                                     desiredControlFramePose,
                                                                     ReferenceFrame.getWorldFrame()));
         long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

         ros2ControllerHelper.publishToController(message);

         trackingCalculator.reset();

         state.setNominalExecutionDuration(getDefinition().getTrajectoryDuration());

         desiredFootPose.setFromReferenceFrame(state.getFootFrame().getReferenceFrame());
         syncedFootPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());
         state.getCommandedTrajectory().setSingleSegmentTrajectory(syncedFootPose, desiredFootPose, getDefinition().getTrajectoryDuration());
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
         state.getLogger().error("Task execution timed out.");
         return;
      }

      if (state.getFootFrame().isChildOfWorld())
      {
         desiredFootPose.setFromReferenceFrame(state.getFootFrame().getReferenceFrame());
         syncedFootPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());

         trackingCalculator.computePoseTrackingData(desiredFootPose, syncedFootPose);

         boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();
         state.getCurrentPose().getValue().set(syncedFootPose);

         if (meetsDesiredCompletionCriteria)
         {
            state.setIsExecuting(false);
         }
      }
   }
}
