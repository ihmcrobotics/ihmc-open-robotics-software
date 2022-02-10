package us.ihmc.footstepPlanning.ui.components;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.HeightMapMessage;
import javafx.animation.AnimationTimer;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.robotEnvironmentAwareness.slam.DriftCorrectionResult;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;
import java.util.UUID;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class HeightMapNavigationUpdater extends AnimationTimer
{
   private final Messager messager;
   private final Stopwatch stopwatch = new Stopwatch();

   private final AtomicReference<Boolean> startHeightMapNavigation;
   private final AtomicReference<Boolean> stopHeightMapNavigation;
   private final AtomicBoolean firstStep = new AtomicBoolean();
   private final AtomicBoolean firstTickInStepState = new AtomicBoolean();

   private final AtomicReference<Point3D> goalPosition;
   private final AtomicReference<Quaternion> goalOrientation;
   private final AtomicReference<HeightMapMessage> heightMapMessage;

   private final FootstepPlannerRequest request = new FootstepPlannerRequest();
   private final FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
   private RobotSide lastStepSide;
   private long previousStepMessageId = 0;

   private final AtomicReference<PlanarRegionsList> planarRegions;
   private final AtomicReference<State> currentState = new AtomicReference<>();
   private final FootstepPlanningModule planningModule;
   private final FootstepPlannerLogger logger;

   private enum State
   {
      WAITING_TO_START,
      PLAN_BODY_PATH,
      FOLLOW_PATH
   }

   public HeightMapNavigationUpdater(Messager messager, WalkingControllerParameters walkingControllerParameters, SideDependentList<List<Point2D>> defaultContactPoints)
   {
      this.messager = messager;
      startHeightMapNavigation = messager.createInput(FootstepPlannerMessagerAPI.StartHeightMapNavigation);
      stopHeightMapNavigation = messager.createInput(FootstepPlannerMessagerAPI.StopHeightMapNavigation);
      goalPosition = messager.createInput(FootstepPlannerMessagerAPI.GoalMidFootPosition);
      goalOrientation = messager.createInput(FootstepPlannerMessagerAPI.GoalMidFootOrientation);
      heightMapMessage = messager.createInput(FootstepPlannerMessagerAPI.HeightMapData);
      planarRegions = messager.createInput(FootstepPlannerMessagerAPI.GPUREARegions);

      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(side ->
                                                                                {
                                                                                   ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
                                                                                   defaultContactPoints.get(side).forEach(defaultFoothold::addVertex);
                                                                                   defaultFoothold.update();
                                                                                   return defaultFoothold;
                                                                                });

      planningModule = new FootstepPlanningModule("HeightMap", new DefaultVisibilityGraphParameters(), new DefaultFootstepPlannerParameters(), new DefaultSwingPlannerParameters(), walkingControllerParameters, footPolygons, null);
      logger = new FootstepPlannerLogger(planningModule);

      currentState.set(State.WAITING_TO_START);
   }

   @Override
   public void handle(long l)
   {
      if (currentState.get() == State.WAITING_TO_START)
      {
         if (!startHeightMapNavigation.getAndSet(false))
         {
            return;
         }

         if (goalPosition.get() == null || goalOrientation.get() == null)
         {
            LogTools.error("Need to set goal before starting.");
            return;
         }
         if (heightMapMessage.get() == null)
         {
            LogTools.error("Height map not received.");
            return;
         }

         reset();
         currentState.set(State.PLAN_BODY_PATH);
      }

      if (currentState.get() == State.PLAN_BODY_PATH)
      {
         // Plan body path
         setStartFootPosesToCurrent();

         request.setHeightMapMessage(heightMapMessage.get());

         Pose3D goalPose = new Pose3D(goalPosition.get(), goalOrientation.get());
         request.setGoalFootPoses(0.2, goalPose);
         request.setPlanBodyPath(true);
         request.setPerformAStarSearch(false);

         LogTools.info("Starting plan");
         FootstepPlannerOutput output = planningModule.handleRequest(request);
         LogTools.info("Finished plan");
         logger.logSession();

         if (output.getBodyPathPlanningResult() != BodyPathPlanningResult.FOUND_SOLUTION)
         {
            LogTools.error("Body path failed to find solution");
            currentState.set(State.WAITING_TO_START);
            reset();
            return;
         }

         request.getBodyPathWaypoints().clear();
         for (int i = 0; i < output.getBodyPath().size(); i++)
         {
            request.getBodyPathWaypoints().add(new Pose3D(output.getBodyPath().get(i)));
         }

         currentState.set(State.FOLLOW_PATH);
      }

      if (currentState.get() == State.FOLLOW_PATH)
      {
         // Execute steps
         if (stopHeightMapNavigation.getAndSet(false))
         {
            LogTools.error("Stop requested");
            currentState.set(State.WAITING_TO_START);
            reset();
            return;
         }
         if (planarRegions.get() == null)
         {
            LogTools.error("No planar regions");
            currentState.set(State.WAITING_TO_START);
            reset();
            return;
         }

         boolean planAStep = firstStep.getAndSet(false) || stopwatch.lapElapsed() > replanDelay;
         if (!planAStep)
         {
            return;
         }

         request.setPlanBodyPath(false);
         request.setPerformAStarSearch(true);
         request.setHeightMapMessage(null);
         request.setPlanarRegionsList(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegions.get()));

         RobotSide stepSide = lastStepSide.getOppositeSide();
         request.setRequestedInitialStanceSide(stepSide);

         if (firstStep.get())
         {
            setStartFootPosesToCurrent();
         }
         else
         {
            setStartFootPosesBasedOnLastCommandedStep();
         }

         LogTools.info("Planning step");
         FootstepPlannerOutput output = planningModule.handleRequest(request);
         FootstepPlanningResult result = output.getFootstepPlanningResult();
         LogTools.info(" \t " + result);
         stopwatch.lap();

         if (result != FootstepPlanningResult.FOUND_SOLUTION)
         {
            return;
         }

         ExecutionMode executionMode = firstStep.get() ? ExecutionMode.OVERRIDE : ExecutionMode.QUEUE;
         PlannedFootstep footstep = output.getFootstepPlan().getFootstep(0);
         footstepDataListMessage.getFootstepDataList().clear();
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().add();
         footstepDataMessage.getLocation().set(footstep.getFootstepPose().getPosition());
         footstepDataMessage.getOrientation().set(footstep.getFootstepPose().getOrientation());

         footstepDataListMessage.getQueueingProperties().setExecutionMode(executionMode.toByte());
         long messageId = UUID.randomUUID().getLeastSignificantBits();
         footstepDataListMessage.getQueueingProperties().setMessageId(messageId);
         footstepDataListMessage.getQueueingProperties().setPreviousMessageId(previousStepMessageId);
         remoteHumanoidInterface.requestWalk(footstepDataListMessage);

         firstStep.set(false);
         lastStepSide = stepSide;
         lastStepPose.set(footstep.getFootstepPose());
         previousStepMessageId = messageId;
         stopwatch.lap();
      }
   }


   private void setStartFootPosesToCurrent()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D foot = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(robotSide));
         foot.changeFrame(ReferenceFrame.getWorldFrame());
         request.setStartFootPose(robotSide, foot);
      }
   }

   private void setStartFootPosesBasedOnLastCommandedStep()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (robotSide == lastStepSide)
         {
            request.setStartFootPose(robotSide, lastStepPose);
         }
         else
         {
            FramePose3D foot = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(robotSide));
            foot.changeFrame(ReferenceFrame.getWorldFrame());
            request.setStartFootPose(robotSide, foot);
         }
      }
   }

   private void reset()
   {
      stopHeightMapNavigation.set(false);
      firstStep.set(true);
      firstTickInStepState.set(true);
      previousStepMessageId = 0;
      lastStepSide = RobotSide.RIGHT;
      stopwatch.start();
   }
}
