package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.NonWallTimer;

import java.util.UUID;

public class FootstepPlanActionExecutorBasics
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final FootstepPlanActionStateBasics state;
   private final FootstepPlanActionDefinitionBasics definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final WalkingFootstepTracker footstepTracker;
   private final WalkingControllerParameters walkingControllerParameters;
   private FootstepPlan footstepPlanToExecute;
   private final SideDependentList<FramePose3D> commandedGoalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> syncedFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<Integer> indexOfLastFoot = new SideDependentList<>();
   private double nominalExecutionDuration;
   private final NonWallTimer executionTimer = new NonWallTimer();
   private final SideDependentList<BehaviorActionCompletionCalculator> completionCalculator = new SideDependentList<>(BehaviorActionCompletionCalculator::new);
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;

   public FootstepPlanActionExecutorBasics(FootstepPlanActionStateBasics state,
                                           FootstepPlanActionDefinitionBasics definition,
                                           ROS2ControllerHelper ros2ControllerHelper,
                                           ROS2SyncedRobotModel syncedRobot,
                                           WalkingFootstepTracker footstepTracker,
                                           WalkingControllerParameters walkingControllerParameters)
   {
      this.state = state;
      this.definition = definition;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.footstepTracker = footstepTracker;
      this.walkingControllerParameters = walkingControllerParameters;
   }

   public void setFootstepPlanToExecute(FootstepPlan footstepPlanToExecute)
   {
      this.footstepPlanToExecute = footstepPlanToExecute;
   }

   public void update()
   {
      executionTimer.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      for (RobotSide side : RobotSide.values)
      {
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
      }
   }

   public void triggerActionExecution()
   {
      if (footstepPlanToExecute.getNumberOfSteps() > 0)
      {
         FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlanToExecute,
                                                                                                                       definition.getSwingDuration(),
                                                                                                                       definition.getTransferDuration());
         footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
         footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
         LogTools.info("Commanding {} footsteps", footstepDataListMessage.getFootstepDataList().size());
         ros2ControllerHelper.publishToController(footstepDataListMessage);
         executionTimer.reset();

         nominalExecutionDuration = PlannerTools.calculateNominalTotalPlanExecutionDuration(footstepPlanToExecute,
                                                                                            definition.getSwingDuration(),
                                                                                            walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                                            definition.getTransferDuration(),
                                                                                            walkingControllerParameters.getDefaultFinalTransferTime());
         for (RobotSide side : RobotSide.values)
         {
            indexOfLastFoot.put(side, -1);
         }
         for (int i = 0; i < footstepPlanToExecute.getNumberOfSteps(); i++)
         {
            indexOfLastFoot.put(footstepPlanToExecute.getFootstep(i).getRobotSide(), i);
         }

         startPositionDistanceToGoal = 0;
         startOrientationDistanceToGoal = 0;
         for (RobotSide side : RobotSide.values)
         {
            int indexOfLastFootSide = indexOfLastFoot.get(side);
            if (indexOfLastFootSide >= 0)
            {
               commandedGoalFeetPoses.get(side).setIncludingFrame(footstepPlanToExecute.getFootstep(indexOfLastFootSide).getFootstepPose());
            }
            else
            {
               commandedGoalFeetPoses.get(side).setIncludingFrame(syncedFeetPoses.get(side));
            }

            startPositionDistanceToGoal += syncedFeetPoses.get(side).getTranslation().differenceNorm(commandedGoalFeetPoses.get(side).getTranslation());
            startOrientationDistanceToGoal += syncedFeetPoses.get(side).getRotation().distance(commandedGoalFeetPoses.get(side).getRotation(), true);
         }
      }
   }

   public void updateCurrentlyExecuting(ActionNodeExecutor<?, ?> actionNodeExecutor)
   {
      if (footstepPlanToExecute != null && footstepPlanToExecute.getNumberOfSteps() > 0)
      {
         boolean isComplete = true;
         for (RobotSide side : RobotSide.values)
         {
            isComplete &= completionCalculator.get(side)
                                              .isComplete(commandedGoalFeetPoses.get(side),
                                                          syncedFeetPoses.get(side),
                                                          POSITION_TOLERANCE,
                                                          ORIENTATION_TOLERANCE,
                                                          nominalExecutionDuration,
                                                          executionTimer,
                                                          actionNodeExecutor.getState(),
                                                          BehaviorActionCompletionComponent.TRANSLATION,
                                                          BehaviorActionCompletionComponent.ORIENTATION);
         }
         int incompleteFootsteps = footstepTracker.getNumberOfIncompleteFootsteps();
         isComplete &= incompleteFootsteps == 0;

         actionNodeExecutor.getState().setIsExecuting(!isComplete);
         actionNodeExecutor.getState().setNominalExecutionDuration(nominalExecutionDuration);
         actionNodeExecutor.getState().setElapsedExecutionTime(executionTimer.getElapsedTime());
         state.setTotalNumberOfFootsteps(footstepPlanToExecute.getNumberOfSteps());
         state.setNumberOfIncompleteFootsteps(incompleteFootsteps);
         actionNodeExecutor.getState().setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
         actionNodeExecutor.getState().setStartPositionDistanceToGoal(startPositionDistanceToGoal);
         actionNodeExecutor.getState().setCurrentOrientationDistanceToGoal(
               completionCalculator.get(RobotSide.LEFT).getRotationError() + completionCalculator.get(RobotSide.RIGHT).getRotationError());
         actionNodeExecutor.getState().setCurrentPositionDistanceToGoal(
               completionCalculator.get(RobotSide.LEFT).getTranslationError() + completionCalculator.get(RobotSide.RIGHT).getTranslationError());
      }
      else
      {
         actionNodeExecutor.getState().setIsExecuting(false);
         actionNodeExecutor.getState().setNominalExecutionDuration(0.0);
         actionNodeExecutor.getState().setElapsedExecutionTime(0.0);
         state.setTotalNumberOfFootsteps(0);
         state.setNumberOfIncompleteFootsteps(0);
         // TODO: Mark action failed and abort execution
      }

      actionNodeExecutor.getState().setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      actionNodeExecutor.getState().setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
   }

   public SideDependentList<Integer> getIndexOfLastFoot()
   {
      return indexOfLastFoot;
   }

   public SideDependentList<FramePose3D> getSyncedFeetPoses()
   {
      return syncedFeetPoses;
   }
}
