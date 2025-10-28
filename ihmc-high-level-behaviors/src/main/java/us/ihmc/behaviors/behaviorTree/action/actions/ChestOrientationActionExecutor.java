package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.TaskspaceTrajectoryTrackingErrorCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;

public class ChestOrientationActionExecutor extends ActionNodeExecutor<ChestOrientationActionState, ChestOrientationActionDefinition>
{
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final FramePose3D desiredChestPose = new FramePose3D();
   private final FramePose3D syncedChestPose = new FramePose3D();
   private final TaskspaceTrajectoryTrackingErrorCalculator trackingCalculator = new TaskspaceTrajectoryTrackingErrorCalculator();

   public ChestOrientationActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new ChestOrientationActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      state.setCanExecute(state.getChestFrame().isChildOfWorld());
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      if (state.getChestFrame().isChildOfWorld())
      {
         FrameQuaternion frameChestQuaternion = new FrameQuaternion(state.getChestFrame().getReferenceFrame());
         frameChestQuaternion.changeFrame(ReferenceFrame.getWorldFrame());

         ChestTrajectoryMessage message = new ChestTrajectoryMessage();
         message.getSo3Trajectory()
                .set(HumanoidMessageTools.createSO3TrajectoryMessage(definition.getTrajectoryDuration(),
                                                                     frameChestQuaternion,
                                                                     EuclidCoreTools.zeroVector3D,
                                                                     ReferenceFrame.getWorldFrame()));
         long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
         message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

         ros2ControllerHelper.publishToController(message);

         trackingCalculator.reset();

         state.setNominalExecutionDuration(definition.getTrajectoryDuration());

         desiredChestPose.setFromReferenceFrame(state.getChestFrame().getReferenceFrame());
         syncedChestPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());
         state.getCommandedTrajectory().setSingleSegmentTrajectory(syncedChestPose, desiredChestPose, definition.getTrajectoryDuration());
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

      if (state.getChestFrame().isChildOfWorld())
      {
         desiredChestPose.setFromReferenceFrame(state.getChestFrame().getReferenceFrame());
         syncedChestPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());

         trackingCalculator.computePoseTrackingData(desiredChestPose, syncedChestPose);
         trackingCalculator.factoryInSO3Errors(ORIENTATION_TOLERANCE);

         boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();
         state.getCurrentPose().accessValue().set(syncedChestPose);
         state.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);

         if (meetsDesiredCompletionCriteria)
         {
            state.setIsExecuting(false);

            if (!definition.getHoldPoseInWorldLater())
            {
               disengageHoldPoseInWorld();
            }
         }

         state.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
      }
   }

   private void disengageHoldPoseInWorld()
   {
      FrameQuaternion frameChestQuaternion = new FrameQuaternion(state.getChestFrame().getReferenceFrame());
      frameChestQuaternion.changeFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());

      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory()
             .set(HumanoidMessageTools.createSO3TrajectoryMessage(definition.getTrajectoryDuration(),
                                                                  frameChestQuaternion,
                                                                  EuclidCoreTools.zeroVector3D,
                                                                  syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame()));
      long frameId = MessageTools.toFrameId(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);
      state.getLogger().info("Publishing chest trajectory message to disengage holding hand in taskspace");
      ros2ControllerHelper.publishToController(message);
   }
}
