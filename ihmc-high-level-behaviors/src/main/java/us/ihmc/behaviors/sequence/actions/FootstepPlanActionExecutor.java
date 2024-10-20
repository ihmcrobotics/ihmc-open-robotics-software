package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.TaskspaceTrajectoryTrackingErrorCalculator;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.thread.Throttler;

import java.util.UUID;

public class FootstepPlanActionExecutor extends ActionNodeExecutor<FootstepPlanActionState, FootstepPlanActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final FootstepPlanActionState state;
   private final FootstepPlanActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ControllerStatusTracker controllerStatusTracker;
   private final WalkingControllerParameters walkingControllerParameters;
   private final SideDependentList<FramePose3D> commandedGoalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> syncedFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<Integer> indexOfLastFoot = new SideDependentList<>();
   private double nominalExecutionDuration;
   private final SideDependentList<TaskspaceTrajectoryTrackingErrorCalculator> trackingCalculators = new SideDependentList<>(
         TaskspaceTrajectoryTrackingErrorCalculator::new);
   private final FramePose3D solePose = new FramePose3D();
   private final FootstepPlan footstepPlanToExecute = new FootstepPlan();
   private final Throttler previewPlanningThrottler = new Throttler().setPeriod(1.0);
   private final FootstepPlanActionPlanningThread previewFootstepPlanningThread;
   private final FootstepPlanActionPlanningThread executionFootstepPlanningThread;
   private final SideDependentList<FramePose3D> liveGoalFeetPoses = new SideDependentList<>(() -> new FramePose3D());

   public FootstepPlanActionExecutor(long id,
                                     CRDTInfo crdtInfo,
                                     WorkspaceResourceDirectory saveFileDirectory,
                                     ROS2ControllerHelper ros2ControllerHelper,
                                     ROS2SyncedRobotModel syncedRobot,
                                     ControllerStatusTracker controllerStatusTracker,
                                     ReferenceFrameLibrary referenceFrameLibrary,
                                     WalkingControllerParameters walkingControllerParameters)
   {
      super(new FootstepPlanActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary, syncedRobot.getRobotModel()));

      state = getState();
      definition = getDefinition();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;
      this.walkingControllerParameters = walkingControllerParameters;

      previewFootstepPlanningThread = new FootstepPlanActionPlanningThread(true, state, definition);
      executionFootstepPlanningThread = new FootstepPlanActionPlanningThread(false, state, definition);
   }

   @Override
   public void update()
   {
      super.update();

      Point3DReadOnly definitionGoalStancePoint = definition.getGoalStancePoint().getValueReadOnly();
      Point3DReadOnly definitionGoalFocalPoint = definition.getGoalFocalPoint().getValueReadOnly();
      boolean invalidDefinition = definitionGoalStancePoint.geometricallyEquals(definitionGoalFocalPoint, 1e-4);

      if (invalidDefinition)
         state.getLogger().error("Approach point can not be in the same place as the focus point.");

      state.setCanExecute(state.areFramesInWorld() && !invalidDefinition);
      if (state.getCanExecute() && !definition.getIsManuallyPlaced())
      {
         FramePoint3D frameStancePoint = new FramePoint3D();
         frameStancePoint.setIncludingFrame(state.getParentFrame(), definitionGoalStancePoint);
         frameStancePoint.changeFrame(ReferenceFrame.getWorldFrame());

         FramePoint3D frameFocalPoint = new FramePoint3D();
         frameFocalPoint.setIncludingFrame(state.getParentFrame(), definitionGoalFocalPoint);
         frameFocalPoint.changeFrame(ReferenceFrame.getWorldFrame());

         double stancePointToFocalPointDistance = frameStancePoint.distance(frameFocalPoint);

         Plane3D zUpPlane = new Plane3D();
         zUpPlane.getPoint().set(frameFocalPoint);
         zUpPlane.getNormal().set(Axis3D.Z);

         Vector3D stancePointVector = new Vector3D();
         stancePointVector.sub(frameStancePoint, frameFocalPoint);
         stancePointVector.normalize();
         if (Math.abs(stancePointVector.getZ()) == 1.0) // This would be undefined
            frameStancePoint.set(frameStancePoint.getZ(), 0.0, 0.0); // Flip to a random direction so we don't crash

         // Project so we can find the horizon level approach point
         zUpPlane.orthogonalProjection(frameStancePoint);

         Vector3D snappedStancePointVector = new Vector3D();
         snappedStancePointVector.sub(frameStancePoint, frameFocalPoint);
         snappedStancePointVector.normalize();
         snappedStancePointVector.scale(stancePointToFocalPointDistance);

         Vector3D snappedFocalPointVector = new Vector3D();
         snappedFocalPointVector.set(snappedStancePointVector);
         snappedFocalPointVector.negate();

         RotationMatrix stanceOrientation = new RotationMatrix();
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.X, snappedFocalPointVector, stanceOrientation);

         FramePoint3D frameSnappedStancePoint = new FramePoint3D();
         frameSnappedStancePoint.setIncludingFrame(frameFocalPoint);
         frameSnappedStancePoint.add(snappedStancePointVector);

         FramePose3D snappedGoalStancePose = new FramePose3D();
         snappedGoalStancePose.getTranslation().set(frameSnappedStancePoint);
         snappedGoalStancePose.getTranslation().setZ(syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame).getZ());
         snappedGoalStancePose.getRotation().set(stanceOrientation);
         snappedGoalStancePose.changeFrame(state.getParentFrame());

         state.getGoalToParentTransform().getValue().set(snappedGoalStancePose);
         state.getGoalFrame().getReferenceFrame().update();

         for (RobotSide side : RobotSide.values)
         {
            state.copyDefinitionToGoalFoostepToGoalTransform(side);

            liveGoalFeetPoses.get(side)
                             .setIncludingFrame(state.getGoalFrame().getReferenceFrame(),
                                                state.getGoalFootstepToGoalTransform(side));
            liveGoalFeetPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
         }

         if (state.getIsNextForExecution())
         {
            if (previewPlanningThrottler.run())
            {
               previewFootstepPlanningThread.triggerPlan(syncedRobot, liveGoalFeetPoses);
            }

            if (previewFootstepPlanningThread.getResultNotification().poll())
            {
               FootstepPlan footstepPlan = previewFootstepPlanningThread.getResultNotification().read();

               var footstepsMessage = state.getPreviewFootsteps().accessValue();
               footstepsMessage.clear();

               for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
               {
                  var messageFootstep = footstepsMessage.add();
                  messageFootstep.setRobotSide(footstepPlan.getFootstep(i).getRobotSide().toByte());
                  messageFootstep.getSolePose().set(footstepPlan.getFootstep(i).getFootstepPose());
               }
            }
         }
      }

      for (RobotSide side : RobotSide.values)
      {
         trackingCalculators.get(side).update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
      }
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      // Reset state
      state.setTotalNumberOfFootsteps(0);
      state.setNumberOfIncompleteFootsteps(0);
      for (RobotSide side : RobotSide.values)
      {
         state.getCurrentFootPoses().get(side).accessValue().set(syncedFeetPoses.get(side));
         state.getDesiredFootPoses().get(side).accessValue().clear();
      }
      state.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      state.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);

      if (state.areFramesInWorld())
      {
         if (definition.getIsManuallyPlaced())
         {
            if (state.getManuallyPlacedFootsteps().isEmpty())
            {
               state.getExecutionState().setValue(FootstepPlanActionExecutionState.PLANNING_FAILED);
            }
            else
            {
               packManuallyPlacedFootstepsIntoPlan();
               state.getExecutionState().setValue(FootstepPlanActionExecutionState.PLANNING_SUCCEEDED);
            }
         }
         else
         {
            executionFootstepPlanningThread.triggerPlan(syncedRobot, liveGoalFeetPoses);
            state.getExecutionState().setValue(FootstepPlanActionExecutionState.FOOTSTEP_PLANNING);
         }
      }
      else
      {
         state.getLogger().error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      switch (state.getExecutionState().getValue())
      {
         case FOOTSTEP_PLANNING ->
         {
            state.setIsExecuting(true);
            // TODO: Maybe report planning elapsed time or something
            if (executionFootstepPlanningThread.planningComplete())
            {
               footstepPlanToExecute.clear();
               footstepPlanToExecute.set(executionFootstepPlanningThread.getResult());
               if (footstepPlanToExecute.isEmpty())
               {
                  state.getExecutionState().setValue(FootstepPlanActionExecutionState.PLANNING_FAILED);
               }
               else
               {
                  state.getExecutionState().setValue(FootstepPlanActionExecutionState.PLANNING_SUCCEEDED);
               }
            }
         }
         case PLANNING_FAILED ->
         {
            state.getLogger().error("No planned steps to execute!");
            state.setIsExecuting(false);
            state.setFailed(true);
         }
         case PLANNING_SUCCEEDED ->
         {
            state.setIsExecuting(true);
            buildAndSendCommandAndSetDesiredState();
            state.getExecutionState().setValue(FootstepPlanActionExecutionState.PLAN_COMMANDED);
         }
         case PLAN_COMMANDED ->
         {
            updateProgress();
         }
      }
   }

   private void packManuallyPlacedFootstepsIntoPlan()
   {
      footstepPlanToExecute.clear();
      for (FootstepPlanActionFootstepState footstep : state.getManuallyPlacedFootsteps())
      {
         solePose.setIncludingFrame(footstep.getSoleFrame().getReferenceFrame().getParent(),
                                    footstep.getDefinition().getSoleToPlanFrameTransform().getValueReadOnly());
         solePose.changeFrame(ReferenceFrame.getWorldFrame());
         footstepPlanToExecute.addFootstep(footstep.getDefinition().getSide(), solePose);
      }
   }

   private void buildAndSendCommandAndSetDesiredState()
   {
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlanToExecute,
                                                                                                                    definition.getSwingDuration(),
                                                                                                                    definition.getTransferDuration());
//      footstepDataListMessage.setTrustHeightOfFootsteps(false); // FIXME: This assumes flat ground
      double finalTransferDuration = 0.01; // We don't want any unecessary pauses at the end; but it can't be 0
      footstepDataListMessage.setFinalTransferDuration(finalTransferDuration);
      footstepDataListMessage.getQueueingProperties().setExecutionMode(definition.getExecutionMode().getValue().toByte());
      footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      state.getLogger().info("Commanding {} footsteps", footstepDataListMessage.getFootstepDataList().size());
      ros2ControllerHelper.publishToController(footstepDataListMessage);
      for (RobotSide side : RobotSide.values)
      {
         trackingCalculators.get(side).reset();
      }

      nominalExecutionDuration = PlannerTools.calculateNominalTotalPlanExecutionDuration(footstepPlanToExecute,
                                                                                         definition.getSwingDuration(),
                                                                                         walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                                         definition.getTransferDuration(),
                                                                                         finalTransferDuration);
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

         state.getDesiredFootPoses().get(side).accessValue().clear();
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

   private void updateProgress()
   {
      boolean hitTimeLimit = false;
      boolean meetsDesiredCompletionCriteria = true;

      for (RobotSide side : RobotSide.values)
      {
         trackingCalculators.get(side).computeExecutionTimings(nominalExecutionDuration);
         trackingCalculators.get(side).computePoseTrackingData(commandedGoalFeetPoses.get(side), syncedFeetPoses.get(side));
         trackingCalculators.get(side).factorInR3Errors(POSITION_TOLERANCE);
         trackingCalculators.get(side).factoryInSO3Errors(ORIENTATION_TOLERANCE);
         meetsDesiredCompletionCriteria &= trackingCalculators.get(side).isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculators.get(side).getTimeIsUp();
         hitTimeLimit |= trackingCalculators.get(side).getHitTimeLimit();
      }

      int incompleteFootsteps = controllerStatusTracker.getFootstepTracker().getNumberOfIncompleteFootsteps();
      boolean isWalking = controllerStatusTracker.isWalking();
      meetsDesiredCompletionCriteria &= incompleteFootsteps == 0;
      meetsDesiredCompletionCriteria &= !isWalking;

      if (meetsDesiredCompletionCriteria || hitTimeLimit)
      {
         state.setIsExecuting(false);
      }
      if (hitTimeLimit)
      {
         state.setFailed(true);
         state.getLogger().info("Walking failed. (time limit)");
      }
      state.setNominalExecutionDuration(nominalExecutionDuration);
      state.setElapsedExecutionTime(trackingCalculators.get(RobotSide.LEFT).getElapsedTime());
      state.setTotalNumberOfFootsteps(footstepPlanToExecute.getNumberOfSteps());
      state.setNumberOfIncompleteFootsteps(incompleteFootsteps);
      for (RobotSide side : RobotSide.values)
      {
         state.getCurrentFootPoses().get(side).accessValue().set(syncedFeetPoses.get(side));
      }
   }
}
