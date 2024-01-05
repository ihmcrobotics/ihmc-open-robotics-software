package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.NonWallTimer;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.UUID;

public class FootstepPlanActionExecutor extends ActionNodeExecutor<FootstepPlanActionState, FootstepPlanActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final FootstepPlanActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final WalkingFootstepTracker footstepTracker;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final WalkingControllerParameters walkingControllerParameters;
   private final FramePose3D solePose = new FramePose3D();
   private final FootstepPlan footstepPlanToExecute = new FootstepPlan();
   private final NonWallTimer executionTimer = new NonWallTimer();
   private final SideDependentList<FramePose3D> goalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> syncedFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<Integer> indexOfLastFoot = new SideDependentList<>();
   private double nominalExecutionDuration;
   private final SideDependentList<BehaviorActionCompletionCalculator> completionCalculator = new SideDependentList<>(BehaviorActionCompletionCalculator::new);
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;

   public FootstepPlanActionExecutor(long id,
                                     CRDTInfo crdtInfo,
                                     WorkspaceResourceDirectory saveFileDirectory,
                                     ROS2ControllerHelper ros2ControllerHelper,
                                     ROS2SyncedRobotModel syncedRobot,
                                     WalkingFootstepTracker footstepTracker,
                                     ReferenceFrameLibrary referenceFrameLibrary,
                                     WalkingControllerParameters walkingControllerParameters)
   {
      super(new FootstepPlanActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.footstepTracker = footstepTracker;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.walkingControllerParameters = walkingControllerParameters;
   }

   @Override
   public void update()
   {
      super.update();

      executionTimer.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      state.setCanExecute(referenceFrameLibrary.containsFrame(getDefinition().getParentFrameName()));
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      if (referenceFrameLibrary.containsFrame(state.getDefinition().getParentFrameName()))
      {
         footstepPlanToExecute.clear();
         for (FootstepPlanActionFootstepState footstep : state.getFootsteps())
         {
            solePose.setIncludingFrame(footstep.getSoleFrame().getReferenceFrame().getParent(),
                                       footstep.getDefinition().getSoleToPlanFrameTransform().getValueReadOnly());
            solePose.changeFrame(ReferenceFrame.getWorldFrame());
            footstepPlanToExecute.addFootstep(footstep.getDefinition().getSide(), solePose);
         }

         FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlanToExecute,
                                                                                                                       getDefinition().getSwingDuration(),
                                                                                                                       getDefinition().getTransferDuration());
         footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
         footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
         ros2ControllerHelper.publishToController(footstepDataListMessage);
         executionTimer.reset();

         nominalExecutionDuration = PlannerTools.calculateNominalTotalPlanExecutionDuration(footstepPlanToExecute,
                                                                                            getDefinition().getSwingDuration(),
                                                                                            walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                                            getDefinition().getTransferDuration(),
                                                                                            walkingControllerParameters.getDefaultFinalTransferTime());

         for (RobotSide side : RobotSide.values)
            indexOfLastFoot.put(side, -1);
         for (int i = 0; i < footstepPlanToExecute.getNumberOfSteps(); i++)
            indexOfLastFoot.put(footstepPlanToExecute.getFootstep(i).getRobotSide(), i);

         for (RobotSide side : RobotSide.values)
         {
            syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));

            int indexOfLastFootSide = indexOfLastFoot.get(side);
            if (indexOfLastFootSide >= 0)
            {
               goalFeetPoses.get(side).setIncludingFrame(footstepPlanToExecute.getFootstep(indexOfLastFootSide).getFootstepPose());
            }
            else
            {
               goalFeetPoses.get(side).setIncludingFrame(syncedFeetPoses.get(side));
            }
         }
         startPositionDistanceToGoal = 0;
         startOrientationDistanceToGoal = 0;
         for (RobotSide side : RobotSide.values)
         {
            startPositionDistanceToGoal += syncedFeetPoses.get(side).getTranslation().differenceNorm(goalFeetPoses.get(side).getTranslation());
            startOrientationDistanceToGoal += syncedFeetPoses.get(side).getRotation().distance(goalFeetPoses.get(side).getRotation(), true);
         }
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      boolean isComplete = true;
      for (RobotSide side : RobotSide.values)
      {
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
         int indexOfLastFootSide = indexOfLastFoot.get(side);
         if (indexOfLastFootSide >= 0)
         {
            goalFeetPoses.get(side).setIncludingFrame(footstepPlanToExecute.getFootstep(indexOfLastFootSide).getFootstepPose());
         }
         else
         {
            goalFeetPoses.get(side).setIncludingFrame(syncedFeetPoses.get(side));
         }

         isComplete &= completionCalculator.get(side)
                                           .isComplete(goalFeetPoses.get(side),
                                                       syncedFeetPoses.get(side),
                                                       POSITION_TOLERANCE,
                                                       ORIENTATION_TOLERANCE,
                                                       nominalExecutionDuration,
                                                       executionTimer,
                                                       getState(),
                                                       BehaviorActionCompletionComponent.TRANSLATION,
                                                       BehaviorActionCompletionComponent.ORIENTATION);
      }
      int incompleteFootsteps = footstepTracker.getNumberOfIncompleteFootsteps();
      isComplete &= incompleteFootsteps == 0;

      state.setIsExecuting(!isComplete);

      state.setNominalExecutionDuration(nominalExecutionDuration);
      state.setElapsedExecutionTime(executionTimer.getElapsedTime());
      state.setTotalNumberOfFootsteps(footstepPlanToExecute.getNumberOfSteps());
      state.setNumberOfIncompleteFootsteps(incompleteFootsteps);
      state.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
      state.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
      state.setCurrentOrientationDistanceToGoal(completionCalculator.get(RobotSide.LEFT).getRotationError()
                                               + completionCalculator.get(RobotSide.RIGHT).getRotationError());
      state.setCurrentPositionDistanceToGoal(completionCalculator.get(RobotSide.LEFT).getTranslationError()
                                           + completionCalculator.get(RobotSide.RIGHT).getTranslationError());
      state.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      state.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
   }
}
