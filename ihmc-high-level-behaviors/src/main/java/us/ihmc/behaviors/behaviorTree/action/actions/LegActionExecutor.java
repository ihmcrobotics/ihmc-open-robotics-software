package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.msg.dds.FootTrajectoryMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.TrajectoryTrackingErrorCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

public class LegActionExecutor extends ActionNodeExecutor<LegActionState, LegActionDefinition>
{
   private final FramePose3D desiredFootPose = new FramePose3D();
   private final FramePose3D syncedFootPose = new FramePose3D();
   private final TrajectoryTrackingErrorCalculator trackingCalculator = new TrajectoryTrackingErrorCalculator();

   public LegActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new LegActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      state.setCanExecute(state.getFootFrame().isChildOfWorld());
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      if (state.getFootFrame().isChildOfWorld())
      {
         FramePose3D desiredControlFramePose = new FramePose3D(state.getFootFrame().getReferenceFrame());
         desiredControlFramePose.changeFrame(ReferenceFrame.getWorldFrame());

         FootTrajectoryMessage message = new FootTrajectoryMessage();
         message.setRobotSide(definition.getSide().toByte());
         message.getSe3Trajectory()
                .set(HumanoidMessageTools.createSE3TrajectoryMessage(definition.getTrajectoryDuration(),
                                                                     desiredControlFramePose,
                                                                     ReferenceFrame.getWorldFrame()));
         long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

         state.getLogger().info("Publishing foot trajectory message");
         ros2ControllerHelper.publishToController(message);

         trackingCalculator.reset();

         state.setNominalExecutionDuration(definition.getTrajectoryDuration());

         desiredFootPose.setFromReferenceFrame(state.getFootFrame().getReferenceFrame());
         syncedFootPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());
         state.getCommandedTrajectory().setSingleSegmentTrajectory(syncedFootPose, desiredFootPose, definition.getTrajectoryDuration());
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

      if (trackingCalculator.getHitTimeLimit(state.getLogger()))
      {
         state.setIsExecuting(false);
         state.setFailed(true);
         return;
      }

      if (state.getFootFrame().isChildOfWorld())
      {
         desiredFootPose.setFromReferenceFrame(state.getFootFrame().getReferenceFrame());
         syncedFootPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());

         trackingCalculator.computePoseTrackingData(desiredFootPose, syncedFootPose);

         boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();
         state.getCurrentPose().accessValue().set(syncedFootPose);

         if (meetsDesiredCompletionCriteria)
         {
            state.setIsExecuting(false);
         }
      }
   }
}
