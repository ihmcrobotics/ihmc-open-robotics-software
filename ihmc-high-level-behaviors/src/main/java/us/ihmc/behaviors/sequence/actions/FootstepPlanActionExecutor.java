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
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.NonWallTimer;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.UUID;

public class FootstepPlanActionExecutor extends ActionNodeExecutor<FootstepPlanActionState, FootstepPlanActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final FootstepPlanActionState state;
   private final FootstepPlanActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final WalkingFootstepTracker footstepTracker;
   private final WalkingControllerParameters walkingControllerParameters;
   private final SideDependentList<FramePose3D> commandedGoalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> syncedFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<Integer> indexOfLastFoot = new SideDependentList<>();
   private double nominalExecutionDuration;
   private final NonWallTimer executionTimer = new NonWallTimer();
   private final SideDependentList<BehaviorActionCompletionCalculator> completionCalculator = new SideDependentList<>(BehaviorActionCompletionCalculator::new);
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final FramePose3D solePose = new FramePose3D();
   private final FootstepPlan footstepPlanToExecute = new FootstepPlan();
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final ResettableExceptionHandlingExecutorService footstepPlanningThread = MissingThreadTools.newSingleThreadExecutor("FootstepPlanning", true, 1);
   private final TypedNotification<FootstepPlan> footstepPlanNotification = new TypedNotification<>();
   private final SideDependentList<FramePose3D> liveGoalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> startFootPosesForThread = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalFootPosesForThread = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   public FootstepPlanActionExecutor(long id,
                                     CRDTInfo crdtInfo,
                                     WorkspaceResourceDirectory saveFileDirectory,
                                     ROS2ControllerHelper ros2ControllerHelper,
                                     ROS2SyncedRobotModel syncedRobot,
                                     WalkingFootstepTracker footstepTracker,
                                     ReferenceFrameLibrary referenceFrameLibrary,
                                     WalkingControllerParameters walkingControllerParameters,
                                     FootstepPlanningModule footstepPlanner,
                                     FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      super(new FootstepPlanActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();
      definition = getDefinition();

      this.referenceFrameLibrary = referenceFrameLibrary;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.footstepTracker = footstepTracker;
      this.walkingControllerParameters = walkingControllerParameters;
      this.footstepPlanner = footstepPlanner;
      this.footstepPlannerParameters = footstepPlannerParameters;
   }

   @Override
   public void update()
   {
      super.update();

      executionTimer.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      for (RobotSide side : RobotSide.values)
      {
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
      }

      state.setCanExecute(referenceFrameLibrary.containsFrame(getDefinition().getParentFrameName()));
      if (state.getCanExecute() && !definition.getIsManuallyPlaced())
      {
         if (state.getGoalFrame().isChildOfWorld())
         {
            for (RobotSide side : RobotSide.values)
            {
               liveGoalFeetPoses.get(side).setIncludingFrame(state.getGoalFrame().getReferenceFrame(),
                                                             getDefinition().getGoalFootstepToGoalTransform(side).getValueReadOnly());
               liveGoalFeetPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
            }
         }
      }
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

               state.getDesiredFootPoses().get(side).getValue().clear();
               state.getDesiredFootPoses().get(side).addTrajectoryPoint(syncedFeetPoses.get(side), 0.0);
            }

            for (int i = 0; i < footstepPlanToExecute.getNumberOfSteps(); i++)
            {
               PlannedFootstep footstep = footstepPlanToExecute.getFootstep(i);
               double stepCompletionTime = PlannerTools.calculateFootstepCompletionTime(footstepPlanToExecute,
                                                                                        definition.getSwingDuration(),
                                                                                        walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                                        definition.getTransferDuration(),
                                                                                        walkingControllerParameters.getDefaultFinalTransferTime(),
                                                                                        i + 1);
               state.getDesiredFootPoses().get(footstep.getRobotSide()).addTrajectoryPoint(footstep.getFootstepPose(), stepCompletionTime);
            }
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
      if (footstepPlanToExecute.getNumberOfSteps() > 0)
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
                                                          state,
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
         for (RobotSide side : RobotSide.values)
         {
            state.getCurrentFootPoses().get(side).getValue().set(syncedFeetPoses.get(side));
         }
      }
      else
      {
         state.setIsExecuting(false);
         state.setNominalExecutionDuration(0.0);
         state.setElapsedExecutionTime(0.0);
         state.setTotalNumberOfFootsteps(0);
         state.setNumberOfIncompleteFootsteps(0);
         for (RobotSide side : RobotSide.values)
         {
            state.getCurrentFootPoses().get(side).getValue().set(syncedFeetPoses.get(side));
            state.getDesiredFootPoses().get(side).getValue().clear();
         }
         // TODO: Mark action failed and abort execution
      }

      state.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      state.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
   }
}
