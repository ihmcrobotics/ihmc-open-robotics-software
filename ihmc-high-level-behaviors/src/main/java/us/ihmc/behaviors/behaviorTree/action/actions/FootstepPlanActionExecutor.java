package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.apache.commons.math3.util.Pair;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.TrajectoryTrackingErrorCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.simplePlanners.QuickFootstepPlanner;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.UUID;

public class FootstepPlanActionExecutor extends ActionNodeExecutor<FootstepPlanActionState, FootstepPlanActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final WalkingControllerParameters walkingControllerParameters;
   private final SideDependentList<FramePose3D> commandedGoalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> syncedFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<Integer> indexOfLastFoot = new SideDependentList<>();
   private double nominalExecutionDuration;
   private final SideDependentList<TrajectoryTrackingErrorCalculator> trackingCalculators = new SideDependentList<>(
         TrajectoryTrackingErrorCalculator::new);
   private final FramePose3D solePose = new FramePose3D();
   private final FootstepPlan footstepPlanToExecute = new FootstepPlan();
   private final Throttler previewPlanningThrottler = new Throttler().setPeriod(1.0);
   private final QuickFootstepPlanner quickFootstepPlanner = new QuickFootstepPlanner();
   private final FootstepPlanActionPlanningThread previewFootstepPlanningThread;
   private final FootstepPlanActionPlanningThread executionFootstepPlanningThread;
   private final SideDependentList<FramePose3D> liveGoalFeetPoses = new SideDependentList<>(() -> new FramePose3D());

   public FootstepPlanActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new FootstepPlanActionState(id, rootNode.getState()), rootNode);

      walkingControllerParameters = robotModel.getWalkingControllerParameters();

      previewFootstepPlanningThread = new FootstepPlanActionPlanningThread(true, state, definition, scene.getTerrainMap());
      executionFootstepPlanningThread = new FootstepPlanActionPlanningThread(false, state, definition, scene.getTerrainMap());
   }

   @Override
   public void update()
   {
      super.update();

      Point3DReadOnly definitionGoalStancePoint = definition.getGoalStancePoint().getValueReadOnly();
      Point3DReadOnly definitionGoalFocalPoint = definition.getGoalFocalPoint().getValueReadOnly();
      boolean stanceEqualsGoal = definitionGoalStancePoint.geometricallyEquals(definitionGoalFocalPoint, 1e-4);

      state.setCanExecute(state.areFramesInWorld() && !stanceEqualsGoal);
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

         state.getGoalToParentTransform().setValue(snappedGoalStancePose, 1e-5);
         state.getGoalFrame().getReferenceFrame().update();

         for (RobotSide side : RobotSide.values)
         {
            state.copyDefinitionToGoalFootstepToGoalTransform(side);

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
      else
      {
         cantExecuteMessage = "";
         if (!state.areFramesInWorld())
         {
            cantExecuteMessage += "state.areFramesInWorld() = false\n";
            cantExecuteMessage += "definition.getParentFrameName() = %s\n".formatted(definition.getParentFrameName());
            cantExecuteMessage += "state.getGoalFrame().isChildOfWorld() = %b\n".formatted(state.getGoalFrame().isChildOfWorld());
         }
         if (!stanceEqualsGoal)
            cantExecuteMessage += "stanceEqualsGoal = false\n";
      }

      for (RobotSide side : RobotSide.values)
      {
         trackingCalculators.get(side).update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
      }
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

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

   private List<Pose3D> computeBodyPathOnly(EnumMap<RobotSide, Pose3D> stanceFeet,
                                            EnumMap<RobotSide, Pose3D> goalFeet)
   {
      FootstepPlannerRequest request = new FootstepPlannerRequest();

      for (RobotSide side : RobotSide.values)
      {
         FramePose3D start = new FramePose3D(ReferenceFrame.getWorldFrame(), stanceFeet.get(side));
         FramePose3D goal  = new FramePose3D(ReferenceFrame.getWorldFrame(), goalFeet.get(side));
         request.setStartFootPose(side, start);
         request.setGoalFootPose(side,  goal);
      }

      request.setPlanBodyPath(true);
      request.setPlanFootsteps(false);
      if (scene.getTerrainMap() != null)
         request.setTerrainMapData(scene.getTerrainMap());

      FootstepPlannerOutput output = executionFootstepPlanningThread.getFootstepPlanner().handleRequest(request, true);
      if (output == null || output.getBodyPath().isEmpty())
      {
         state.getLogger().warn("Body path planning failed or returned empty path. Falling back to straight-line quick planner.");
         return null; // use null to mean "no body path"
      }

      return new ArrayList<>(output.getBodyPath());
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
         hitTimeLimit |= trackingCalculators.get(side).getHitTimeLimit(state.getLogger());
      }

      int incompleteFootsteps = controllerStatusTracker.getFootstepTracker().getNumberOfIncompleteFootsteps();
      boolean isWalking = controllerStatusTracker.isWalking();
      meetsDesiredCompletionCriteria &= incompleteFootsteps == 0;
      meetsDesiredCompletionCriteria &= !isWalking;

      if (meetsDesiredCompletionCriteria || hitTimeLimit || state.getFailed())
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
