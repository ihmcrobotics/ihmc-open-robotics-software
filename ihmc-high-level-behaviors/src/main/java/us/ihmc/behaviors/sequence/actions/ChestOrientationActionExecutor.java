package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.TrajectoryTrackingErrorCalculator;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ChestOrientationActionExecutor extends ActionNodeExecutor<ChestOrientationActionState, ChestOrientationActionDefinition>
{
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ChestOrientationActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final TrajectoryActionExecutorCommonFunctionality trajectoryActionCommon;
   private final FramePose3D desiredChestPose = new FramePose3D();
   private final FramePose3D syncedChestPose = new FramePose3D();
   private double startOrientationDistanceToGoal;
   private final TrajectoryTrackingErrorCalculator trackingCalculator = new TrajectoryTrackingErrorCalculator();

   public ChestOrientationActionExecutor(long id,
                                         CRDTInfo crdtInfo,
                                         WorkspaceResourceDirectory saveFileDirectory,
                                         ROS2ControllerHelper ros2ControllerHelper,
                                         ROS2SyncedRobotModel syncedRobot,
                                         ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(new ChestOrientationActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;

      trajectoryActionCommon = new TrajectoryActionExecutorCommonFunctionality(syncedRobot, ros2ControllerHelper, state);
   }

   @Override
   public void update()
   {
      super.update();

      trajectoryActionCommon.update();

      state.setCanExecute(state.getChestFrame().isChildOfWorld());
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      if (state.getChestFrame().isChildOfWorld())
      {
         FrameQuaternion frameChestQuaternion = new FrameQuaternion(state.getChestFrame().getReferenceFrame());
         frameChestQuaternion.changeFrame(ReferenceFrame.getWorldFrame());

         ChestTrajectoryMessage message = new ChestTrajectoryMessage();
         message.getSo3Trajectory()
                .set(HumanoidMessageTools.createSO3TrajectoryMessage(getDefinition().getTrajectoryDuration(),
                                                                     frameChestQuaternion,
                                                                     EuclidCoreTools.zeroVector3D,
                                                                     ReferenceFrame.getWorldFrame()));
         long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
         message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

         ros2ControllerHelper.publishToController(message);

         trajectoryActionCommon.triggerActionExecution(getDefinition().getTrajectoryDuration());

         desiredChestPose.setFromReferenceFrame(state.getChestFrame().getReferenceFrame());
         syncedChestPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());
         startOrientationDistanceToGoal = syncedChestPose.getRotation().distance(desiredChestPose.getRotation(), true);
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      trajectoryActionCommon.checkAndHandleTimeLimit();

      if (!trajectoryActionCommon.getHitTimeLimit() && state.getChestFrame().isChildOfWorld())
      {
         desiredChestPose.setFromReferenceFrame(state.getChestFrame().getReferenceFrame());
         syncedChestPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame());

         trackingCalculator.computePoseTrackingData(desiredChestPose, syncedChestPose);
         trackingCalculator.factoryInSO3Errors(ORIENTATION_TOLERANCE);

         boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();

         if (meetsDesiredCompletionCriteria)
         {
            state.setIsExecuting(false);

            if (!getDefinition().getHoldPoseInWorldLater())
            {
               disengageHoldPoseInWorld();
            }
         }

         state.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
         state.setCurrentOrientationDistanceToGoal(trackingCalculator.getOrientationError());
         state.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
      }
   }

   private void disengageHoldPoseInWorld()
   {
      FrameQuaternion frameChestQuaternion = new FrameQuaternion(state.getChestFrame().getReferenceFrame());
      frameChestQuaternion.changeFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());

      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory()
             .set(HumanoidMessageTools.createSO3TrajectoryMessage(getDefinition().getTrajectoryDuration(),
                                                                  frameChestQuaternion,
                                                                  EuclidCoreTools.zeroVector3D,
                                                                  syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame()));
      long frameId = MessageTools.toFrameId(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);
      LogTools.info("Publishing chest trajectory message to disengage holding hand in taskspace");
      ros2ControllerHelper.publishToController(message);
   }
}
