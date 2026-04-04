package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.TrajectoryTrackingErrorCalculator;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectType;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.simplePlanners.QuickFootstepPlanner;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.rrt.RRTConnectPathPlanner;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDAShapePointCounter;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

import static behavior_msgs.msg.dds.WalkActionDefinitionMessage.*;
import static us.ihmc.footstepPlanning.simplePlanners.QuickFootstepPlanner.Footstep;

public class WalkActionExecutor extends ActionNodeExecutor<WalkActionState, WalkActionDefinition>
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
   private final RRTConnectPathPlanner rrtConnectPathPlanner = new RRTConnectPathPlanner();
   private volatile CUDAShapePointCounter shapePointCounter;
   private boolean startedCreatingPlanners = false;
   private WalkActionPlanningThread previewFootstepPlanningThread;
   private WalkActionPlanningThread executionFootstepPlanningThread;
   private final SideDependentList<FramePose3D> liveGoalFeetPoses = new SideDependentList<>(() -> new FramePose3D());

   public WalkActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new WalkActionState(id, rootNode.getState()), rootNode);

      walkingControllerParameters = robotModel.getWalkingControllerParameters();

      ThreadTools.startAsDaemon(() -> shapePointCounter = new CUDAShapePointCounter(), "CreateShapePointCounter");
   }

   @Override
   public void update()
   {
      super.update();

      Point3DReadOnly definitionGoalStancePoint = definition.getGoalStancePoint().getValueReadOnly();
      Point3DReadOnly definitionGoalFocalPoint = definition.getGoalFocalPoint().getValueReadOnly();
      boolean stanceEqualsFocal = definitionGoalStancePoint.geometricallyEquals(definitionGoalFocalPoint, 1e-4);

      if (definition.getPlannerType().getValue() != QUICK && !startedCreatingPlanners)
      {
         startedCreatingPlanners = true;
         ThreadTools.startAsDaemon(() ->
         {
            WalkActionPlanningThread previewFootstepPlanningThread = new WalkActionPlanningThread(true, state, definition, scene.getTerrainMap());
            WalkActionPlanningThread executionFootstepPlanningThread = new WalkActionPlanningThread(false, state, definition, scene.getTerrainMap());
            this.executionFootstepPlanningThread = executionFootstepPlanningThread;
            this.previewFootstepPlanningThread = previewFootstepPlanningThread; // Set this last because it's the check that it's complete
         }, "CreatePlanners");
      }

      boolean canExecute = state.areFramesInWorld();
      canExecute &= !stanceEqualsFocal;
      boolean plannersReady = definition.getPlannerType().getValue() == QUICK || previewFootstepPlanningThread != null;
      canExecute &= plannersReady;
      state.setCanExecute(canExecute);
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
            if (definition.getPlannerType().getValue() == QUICK)
            {
               if (previewPlanningThrottler.run())
               {
                  List<Footstep> footstepPlan = planQuickFootsteps();

                  var footstepsMessage = state.getPreviewFootsteps().accessValue();
                  footstepsMessage.clear();
                  for (Footstep footstep : footstepPlan)
                  {
                     var messageFootstep = footstepsMessage.add();
                     messageFootstep.setRobotSide(footstep.swingSide().toByte());
                     messageFootstep.getSolePose().set(footstep.swingEnd());
                  }
               }
            }
            else if (previewFootstepPlanningThread != null)
            {
               if (previewPlanningThrottler.run())
                  previewFootstepPlanningThread.triggerPlan(syncedRobot, liveGoalFeetPoses);

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
         if (!stanceEqualsFocal)
            cantExecuteMessage += "stanceEqualsFocal = false\n";
         if (!plannersReady)
            cantExecuteMessage += "planners still initializing\n";
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

      boolean plannersReady = definition.getPlannerType().getValue() == QUICK || previewFootstepPlanningThread != null;
      if (state.areFramesInWorld() && plannersReady)
      {
         if (definition.getIsManuallyPlaced())
         {
            if (state.getManuallyPlacedFootsteps().isEmpty())
            {
               state.getExecutionState().setValue(WalkActionExecutionState.PLANNING_FAILED);
            }
            else
            {
               packManuallyPlacedFootstepsIntoPlan();
               updatePreviewFootstepsFromPlan(footstepPlanToExecute);
               state.getExecutionState().setValue(WalkActionExecutionState.PLANNING_SUCCEEDED);
            }
         }
         else if (definition.getPlannerType().getValue() == QUICK)
         {
            List<Footstep> footstepPlan = planQuickFootsteps();

            footstepPlanToExecute.clear();
            for (int i = 0; i < footstepPlan.size(); i++)
            {
               Footstep quickFootstep = footstepPlan.get(i);
               PlannedFootstep simpleFootstep = new PlannedFootstep(quickFootstep.swingSide(), new FramePose3D(quickFootstep.swingEnd()));
               // Increase swing duration for longer steps
               double minDistance = definition.getQuickSwingTimeDistanceLower().getValue();
               double maxDistance = definition.getQuickSwingTimeDistanceUpper().getValue();
               double minTime = definition.getQuickMinSwingTime().getValue();
               double maxTime = definition.getQuickMaxSwingTime().getValue();
               double distance = quickFootstep.swingDistance();
               double time = (distance <= minDistance) ? minTime : // Min
                                 (distance >= maxDistance) ? maxTime : // Max
                                      minTime + ((distance - minDistance) / (maxDistance - minDistance)) * (maxTime - minTime); // Interpolate
               LogTools.info("Swing time: %.2f".formatted(time));
               simpleFootstep.setSwingDuration(time);
               footstepPlanToExecute.addFootstep(simpleFootstep);
            }
            updatePreviewFootstepsFromPlan(footstepPlanToExecute);
            state.getExecutionState().setValue(WalkActionExecutionState.PLANNING_SUCCEEDED);
         }
         else
         {
            executionFootstepPlanningThread.triggerPlan(syncedRobot, liveGoalFeetPoses);
            state.getExecutionState().setValue(WalkActionExecutionState.FOOTSTEP_PLANNING);
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
                  state.getExecutionState().setValue(WalkActionExecutionState.PLANNING_FAILED);
               }
               else
               {
                  updatePreviewFootstepsFromPlan(footstepPlanToExecute);
                  state.getExecutionState().setValue(WalkActionExecutionState.PLANNING_SUCCEEDED);
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
            state.getExecutionState().setValue(WalkActionExecutionState.PLAN_COMMANDED);
         }
         case PLAN_COMMANDED ->
         {
            updateProgress();
         }
      }
   }

   @Override
   public void destroy()
   {
      super.destroy();

      if (shapePointCounter != null)
         shapePointCounter.close();
   }

   private void packManuallyPlacedFootstepsIntoPlan()
   {
      footstepPlanToExecute.clear();
      for (WalkActionFootstepState footstep : state.getManuallyPlacedFootsteps())
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

   private void updatePreviewFootstepsFromPlan(FootstepPlan plan)
   {
      var footstepsMessage = state.getPreviewFootsteps().accessValue();
      footstepsMessage.clear();
      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         var messageFootstep = footstepsMessage.add();
         messageFootstep.setRobotSide(plan.getFootstep(i).getRobotSide().toByte());
         messageFootstep.getSolePose().set(plan.getFootstep(i).getFootstepPose());
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

   private List<Footstep> planQuickFootsteps()
   {
      quickFootstepPlanner.setHipWidth(definition.getQuickHipWidth().getValue());
      quickFootstepPlanner.setStepLength(definition.getQuickStepLength().getValue());
      quickFootstepPlanner.setNextPelvisYawLimit(definition.getQuickNextPelvisYawLimit().getValue());
      quickFootstepPlanner.setInwardLimit(definition.getQuickInwardLimit().getValue());
      quickFootstepPlanner.setOutwardLimit(definition.getQuickOutwardLimit().getValue());
      quickFootstepPlanner.setStepAngleLimit(definition.getQuickStepAngleLimit().getValue());
      ArrayList<Pose3D> waypoints = new ArrayList<>();
      Pose3D midGoal = new Pose3D();
      midGoal.interpolate(liveGoalFeetPoses.get(RobotSide.LEFT), liveGoalFeetPoses.get(RobotSide.RIGHT), 0.5);
      if (definition.getUseRRTPathPlanner().getValue())
      {
         Point3D midStance = new Point3D();
         midStance.interpolate(syncedFeetPoses.get(RobotSide.LEFT).getPosition(), syncedFeetPoses.get(RobotSide.RIGHT).getPosition(), 0.5);
         Point3D capsuleBottom = new Point3D();
         Point3D capsuleTop = new Point3D();
         List<Point3D> path = rrtConnectPathPlanner.plan(midStance, midGoal.getPosition(), segment ->
         {
            RawImage depthImage;
            ImageSensor imageSensor = scene.getImageSensor();
            if (shapePointCounter != null && imageSensor != null && (depthImage = imageSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY)) != null)
            {
               capsuleBottom.set(segment.getFirstEndpoint());
               capsuleBottom.addZ(0.5);
               capsuleTop.set(capsuleBottom);
               capsuleTop.addZ(1.0);
               if (shapePointCounter.countPointsInCapsule(depthImage, capsuleBottom, capsuleTop, (float) definition.getObstacleClearanceRadius().getValue()) > 0)
               {
                  depthImage.release();
                  return true;
               }

               capsuleBottom.set(segment.getSecondEndpoint());
               capsuleBottom.addZ(0.5);
               capsuleTop.set(capsuleBottom);
               capsuleTop.addZ(1.0);
               if (shapePointCounter.countPointsInCapsule(depthImage, capsuleBottom, capsuleTop, (float) definition.getObstacleClearanceRadius().getValue()) > 0)
               {
                  depthImage.release();
                  return true;
               }

               depthImage.release();
            }

            return false;
         });

         double segmentLength = 0.0;
         for (int i = 1; i < path.size(); i++)
         {
            segmentLength += path.get(i - 1).distance(path.get(i));
            if (segmentLength >= 0.4 && path.get(i).distance(midGoal.getPosition()) > 0.2) // Don't add a waypoint too close to goal
            {
               double yaw = Math.atan2(path.get(i).getY() - path.get(i - 1).getY(), path.get(i).getX() - path.get(i - 1).getX());
               Pose3D waypoint = new Pose3D();
               waypoint.getPosition().set(path.get(i));
               waypoint.getOrientation().setToYawOrientation(yaw);
               waypoints.add(waypoint);
               segmentLength = 0.0;
            }
         }
      }
      else
      {
         for (int i = 0; i < definition.getWaypoints().getSize(); i++)
         {
            FramePose3D framePose = new FramePose3D(state.getParentFrame(), definition.getWaypoints().getValueReadOnly(i));
            framePose.changeFrame(ReferenceFrame.getWorldFrame());
            waypoints.add(new Pose3D(framePose));
         }
      }
      if (definition.getQuickWaypointOnly().getValue())
         waypoints.add(midGoal);
      return quickFootstepPlanner.plan(new SideDependentList<>(side -> new Pose3D(syncedFeetPoses.get(side))),
                                       waypoints,
                                       definition.getQuickWaypointOnly().getValue() ? null
                                          : new SideDependentList<>(side -> new Pose3D(liveGoalFeetPoses.get(side))));
   }
}
