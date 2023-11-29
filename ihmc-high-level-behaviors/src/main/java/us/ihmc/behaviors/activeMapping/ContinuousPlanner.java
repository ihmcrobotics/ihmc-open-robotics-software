package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.monteCarloPlanning.*;
import us.ihmc.footstepPlanning.swing.CollisionFreeSwingCalculator;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class ContinuousPlanner
{
   private static final boolean LOG_FOOTSTEP_PLANS = false;

   public enum PlanningMode
   {
      EXECUTE_AND_PAUSE, FRONTIER_EXPANSION, ACTIVE_SEARCH, WALK_TO_GOAL, RANDOM_WALK, INCREMENTAL_PLANNING
   }

   private PlanningMode mode = PlanningMode.WALK_TO_GOAL;

   private ContinuousGoalGenerator goalGenerator = new ContinuousGoalGenerator(0.0, 5.0, 0.0, 5.0);
   private FramePose3D walkingStartMidPose = new FramePose3D();
   private final SideDependentList<FramePose3D> goalStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> startingStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private FramePose3D imminentFootstepPose = new FramePose3D();
   private RobotSide imminentFootstepSide = RobotSide.LEFT;

   private final Mat gridColor = new Mat();

   private final Point2D gridOrigin = new Point2D(0.0, -1.0);
   private final Point2D goalPosition = new Point2D();
   private final Point2D goalPositionIndices = new Point2D();
   private final Point2D agentPosition = new Point2D();
   private final Point2D agentPositionIndices = new Point2D();
   private final Point2D robotLocation = new Point2D();
   private final Point2D robotLocationIndices = new Point2D();

   private final HumanoidReferenceFrames referenceFrames;
   private CollisionFreeSwingCalculator collisionFreeSwingCalculator;
   private ContinuousWalkingParameters continuousPlanningParameters;
   private ContinuousPlannerStatistics statistics;
   private SwingPlannerParametersBasics swingPlannerParameters;
   private FootstepPlanningModule footstepPlanner;
   private FootstepPlanningResult footstepPlanningResult;
   private FootstepPlan previousFootstepPlan;
   private FootstepPlan latestFootstepPlan;
   private List<EnumMap<Axis3D, List<PolynomialReadOnly>>> latestSwingTrajectories;
   private FootstepPlannerLogger logger;
   private MonteCarloPathPlanner monteCarloPathPlanner;
   private MonteCarloFootstepPlannerParameters monteCarloFootstepPlannerParameters;
   private MonteCarloFootstepPlanner monteCarloFootstepPlanner;

   private boolean initialized = false;
   private boolean planAvailable = false;
   private boolean active;

   private float gridResolution = 10.0f;
   private int offset = 70;

   public ContinuousPlanner(DRCRobotModel robotModel,
                            HumanoidReferenceFrames humanoidReferenceFrames,
                            ContinuousPlannerStatistics statistics,
                            SwingPlannerParametersBasics swingPlannerParameters,
                            PlanningMode mode)
   {
      this.referenceFrames = humanoidReferenceFrames;
      this.mode = mode;
      this.statistics = statistics;
      this.swingPlannerParameters = swingPlannerParameters;

      active = true;

      switch (mode)
      {
         case FRONTIER_EXPANSION:
            monteCarloPathPlanner = new MonteCarloPathPlanner(offset);
            break;
         case WALK_TO_GOAL, RANDOM_WALK:
            footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel, "ForContinuousWalking");
            logger = new FootstepPlannerLogger(footstepPlanner);
            break;
         case INCREMENTAL_PLANNING:
            monteCarloFootstepPlannerParameters = new MonteCarloFootstepPlannerParameters();
            monteCarloFootstepPlanner = new MonteCarloFootstepPlanner(monteCarloFootstepPlannerParameters);
            collisionFreeSwingCalculator = new CollisionFreeSwingCalculator(robotModel.getFootstepPlannerParameters("ForContinuousWalking"),
                                                                            robotModel.getSwingPlannerParameters(),
                                                                            robotModel.getWalkingControllerParameters(),
                                                                            FootstepPlanningModuleLauncher.createFootPolygons(robotModel));
            break;
      }
   }

   public void initialize()
   {
      switch (mode)
      {
         case WALK_TO_GOAL ->
         {
            footstepPlanner.clearCustomTerminationConditions();
            footstepPlanner.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) -> pathSize
                                                                                                                        >= continuousPlanningParameters.getNumberOfStepsToSend());
         }
      }

      for (RobotSide side : RobotSide.values)
      {
         startingStancePose.get(side).setFromReferenceFrame(referenceFrames.getSoleFrame(side));
         goalStancePose.get(side).setFromReferenceFrame(referenceFrames.getSoleFrame(side));
      }

      FramePose3D finalGoalMidPose = new FramePose3D();
      finalGoalMidPose.interpolate(startingStancePose.get(RobotSide.LEFT), startingStancePose.get(RobotSide.RIGHT), 0.5);

      walkingStartMidPose.getPosition().setX(finalGoalMidPose.getPosition().getX());
      walkingStartMidPose.getPosition().setY(finalGoalMidPose.getPosition().getY());
      walkingStartMidPose.getPosition().setZ(finalGoalMidPose.getPosition().getZ());
      walkingStartMidPose.getOrientation().setToYawOrientation(finalGoalMidPose.getRotation().getYaw());

      initialized = true;
   }

   public void setGoalWaypointPoses(ContinuousWalkingParameters continuousPlanningParameters)
   {
      this.continuousPlanningParameters = continuousPlanningParameters;

      switch (this.mode)
      {
         case WALK_TO_GOAL, INCREMENTAL_PLANNING:
            ContinuousPlanningTools.setRandomizedStraightGoalPoses(walkingStartMidPose,
                                                                   startingStancePose,
                                                                   goalStancePose,
                                                                   (float) continuousPlanningParameters.getGoalPoseForwardDistance(),
                                                                   (float) continuousPlanningParameters.getGoalPoseUpDistance());
            break;
         case RANDOM_WALK:
            goalGenerator.updateCurrentPosition(new Point3D(startingStancePose.get(RobotSide.LEFT).getPosition()));
            ContinuousPlanningTools.setRandomGoalWithinBounds(goalGenerator.getNextLocation(),
                                                              startingStancePose,
                                                              goalStancePose,
                                                              (float) continuousPlanningParameters.getGoalPoseForwardDistance(),
                                                              (float) continuousPlanningParameters.getGoalPoseUpDistance());
            break;
         case FRONTIER_EXPANSION:
            //Pose2D goalPose2D = ActiveMappingTools.getNearestUnexploredNode(planarRegionMap.getMapRegions(), gridOrigin, robotPose2D, gridSize, gridResolution);
            break;
         case ACTIVE_SEARCH:
         {
            robotLocation.set((startingStancePose.get(RobotSide.LEFT).getX() + startingStancePose.get(RobotSide.RIGHT).getX()) / 2.0f,
                              (startingStancePose.get(RobotSide.LEFT).getY() + startingStancePose.get(RobotSide.RIGHT).getY()) / 2.0f);

            monteCarloPathPlanner.getAgent().measure(monteCarloPathPlanner.getWorld());
            agentPositionIndices.set(monteCarloPathPlanner.getAgent().getState());
            robotLocationIndices.set(HeightMapTools.getIndexFromCoordinates(robotLocation.getX(), gridResolution, offset),
                                     HeightMapTools.getIndexFromCoordinates(robotLocation.getY(), gridResolution, offset));

            double error = agentPositionIndices.distance(robotLocationIndices);

            if (error < 10.0f)
            {
               goalPositionIndices.set((Point2DReadOnly) monteCarloPathPlanner.plan().getState());
               goalPosition.set(HeightMapTools.getCoordinateFromIndex((int) goalPositionIndices.getX(), gridResolution, offset),
                                HeightMapTools.getCoordinateFromIndex((int) goalPositionIndices.getY(), gridResolution, offset));

               monteCarloPathPlanner.updateState(new MonteCarloWaypointNode(goalPositionIndices, null, 0));
            }

            float yawRobotToGoal = (float) Math.atan2(goalPosition.getY() - robotLocation.getY(), goalPosition.getX() - robotLocation.getX());

            LogTools.info("Next Goal: Position:{}, Yaw{}", goalPosition, yawRobotToGoal);

            Pose3D goalPoseToUse = new Pose3D(goalPosition.getX(), goalPosition.getY(), 0.0, 0.0, 0.0, yawRobotToGoal);
            goalStancePose.get(RobotSide.LEFT).set(goalPoseToUse);
            goalStancePose.get(RobotSide.RIGHT).set(goalPoseToUse);
            goalStancePose.get(RobotSide.LEFT).prependTranslation(0.0, 0.12, 0.0);
            goalStancePose.get(RobotSide.RIGHT).prependTranslation(0.0, -0.12, 0.0);
         }
      }
   }

   public void planToGoalWithHeightMap(HeightMapData heightMapData, Mat heightMap, Mat contactMap, boolean useReferencePlan)
   {
      long startTime = System.currentTimeMillis();

      switch (mode)
      {
         case WALK_TO_GOAL, RANDOM_WALK -> planWithAStarPlanner(heightMapData, useReferencePlan);
         case INCREMENTAL_PLANNING -> planWithMonteCarloPlanner(heightMapData, heightMap, contactMap, false);
      }

      statistics.setLastPlanningTime((float) (System.currentTimeMillis() - startTime) / 1000.0f);
   }

   public void planWithMonteCarloPlanner(HeightMapData heightMapData, Mat heightMapImage, Mat contactMapImage, boolean reset)
   {
      if (monteCarloFootstepPlanner.isPlanning())
      {
         LogTools.warn("Monte Carlo Footstep Planner is Busy!");
         return;
      }

      MonteCarloFootstepPlannerRequest request = new MonteCarloFootstepPlannerRequest();
      request.setStartFootPose(RobotSide.LEFT, startingStancePose.get(RobotSide.LEFT));
      request.setStartFootPose(RobotSide.RIGHT, startingStancePose.get(RobotSide.RIGHT));
      request.setGoalFootPose(RobotSide.LEFT, goalStancePose.get(RobotSide.LEFT));
      request.setGoalFootPose(RobotSide.RIGHT, goalStancePose.get(RobotSide.RIGHT));
      request.setContactMap(contactMapImage);
      request.setHeightMap(heightMapImage);

      long timeStart = System.nanoTime();

      if (reset)
         monteCarloFootstepPlanner.reset(request);

      latestFootstepPlan = monteCarloFootstepPlanner.generateFootstepPlan(request);

      collisionFreeSwingCalculator.setHeightMapData(heightMapData);
      collisionFreeSwingCalculator.computeSwingTrajectories(startingStancePose, latestFootstepPlan);

      latestSwingTrajectories = collisionFreeSwingCalculator.getSwingTrajectories();

      footstepPlanningResult = FootstepPlanningResult.FOUND_SOLUTION;
      planAvailable = latestFootstepPlan.getNumberOfSteps() > 0;

      long timeEnd = System.nanoTime();

      LogTools.info(String.format("Total Time: %.3f ms, Plan Size: %d, Visited: %d, Layer Counts: %s",
                                  (timeEnd - timeStart) / 1e6,
                                  latestFootstepPlan.getNumberOfSteps(),
                                  monteCarloFootstepPlanner.getVisitedNodes().size(),
                                  MonteCarloPlannerTools.getLayerCountsString(monteCarloFootstepPlanner.getRoot())));

      monteCarloFootstepPlanner.getDebugger().plotFootstepPlan(latestFootstepPlan);
      monteCarloFootstepPlanner.getDebugger().display(1);
   }

   public void planWithAStarPlanner(HeightMapData heightMapData, boolean useReferencePlan)
   {
      if (footstepPlanner.isPlanning())
      {
         LogTools.warn("Footstep Planner is Busy!");
         return;
      }

      // Sync the swing time to always be the same whether its obstacle avoidance or not
      swingPlannerParameters.setMinimumSwingTime(continuousPlanningParameters.getSwingTime());
      swingPlannerParameters.setMaximumSwingTime(continuousPlanningParameters.getSwingTime());

      footstepPlanner.getSwingPlannerParameters().set(swingPlannerParameters);

      FootstepPlannerRequest request = createFootstepPlannerRequest(startingStancePose, goalStancePose);
      request.setRequestedInitialStanceSide(imminentFootstepSide);
      request.setSnapGoalSteps(true);
      request.setHeightMapData(heightMapData);
      request.setAbortIfGoalStepSnappingFails(true);

      if (useReferencePlan)
      {
         // Sets the previous footstep plan to be a reference for the current plan
         FootstepPlan previousFootstepPlan = new FootstepPlan(latestFootstepPlan);

         if (previousFootstepPlan.getNumberOfSteps() < continuousPlanningParameters.getNumberOfStepsToSend())
         {
            LogTools.error("Previous Plan for Reference: Not Enough Steps: {}!", previousFootstepPlan.getNumberOfSteps());
         }
         else
         {
            // These are steps that are considered to be at the start of the plan, don't want to use them as reference
            this.previousFootstepPlan.remove(0);

            if (!continuousPlanningParameters.getOverrideEntireQueueEachStep())
               this.previousFootstepPlan.remove(1);

            request.setReferencePlan(this.previousFootstepPlan);
            request.setTimeout(continuousPlanningParameters.getPlanningReferenceTimeout());
         }
      }
      else
      {
         request.setTimeout(continuousPlanningParameters.getPlanningWithoutReferenceTimeout());
      }

      FootstepPlannerOutput plannerOutput = footstepPlanner.handleRequest(request);

      if (plannerOutput != null)
      {
         footstepPlanningResult = plannerOutput.getFootstepPlanningResult();
         planAvailable = footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps() > 0;
         latestFootstepPlan = plannerOutput.getFootstepPlan();
         latestSwingTrajectories = plannerOutput.getSwingTrajectories();

         LogTools.info(String.format("Plan Result: %s, Steps: %d, Result: %s, Initial Stance: %s",
                                     footstepPlanningResult,
                                     footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps(),
                                     planAvailable,
                                     imminentFootstepSide));
      }

      if (LOG_FOOTSTEP_PLANS)
      {
         ThreadTools.startAThread(() ->
                                  {
                                     LogTools.info("Logging Session");
                                     logger.logSession();
                                     FootstepPlannerLogger.deleteOldLogs();
                                  }, "FootstepPlanLogAndDeletion");
      }
   }

   public FootstepPlan planToExploreWithHeightMap(HeightMapData heightMapData)
   {
      MonteCarloPlannerTools.plotWorld(monteCarloPathPlanner.getWorld(), gridColor);
      MonteCarloPlannerTools.plotAgent(monteCarloPathPlanner.getAgent(), gridColor);
      MonteCarloPlannerTools.plotRangeScan(monteCarloPathPlanner.getAgent().getScanPoints(), gridColor);

      PerceptionDebugTools.display("Monte Carlo Planner World", gridColor, 1, 1400);

      if (active)
      {
         LogTools.info("Footstep Planning Request");

         FootstepPlannerRequest request = createFootstepPlannerRequest(startingStancePose, goalStancePose);
         request.setHeightMapData(heightMapData);
         FootstepPlannerOutput plannerOutput = footstepPlanner.handleRequest(request);

         if (plannerOutput != null)
         {
            FootstepPlanningResult footstepPlanningResult = plannerOutput.getFootstepPlanningResult();
            LogTools.info("Footstep Planning Result: {}", footstepPlanningResult);
            LogTools.info(String.format("Plan Length: %d\n", footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps()));
            planAvailable = footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps() > 0;
         }
      }
      return latestFootstepPlan;
   }

   public void planToGoalWithPlanarRegionMap(PlanarRegionMap planarRegionMap)
   {
      if (mode == PlanningMode.FRONTIER_EXPANSION)
      {
         MonteCarloPlannerTools.plotWorld(monteCarloPathPlanner.getWorld(), gridColor);
         MonteCarloPlannerTools.plotAgent(monteCarloPathPlanner.getAgent(), gridColor);
         MonteCarloPlannerTools.plotRangeScan(monteCarloPathPlanner.getAgent().getScanPoints(), gridColor);

         PerceptionDebugTools.display("Monte Carlo Planner World", gridColor, 1, 1400);

         if (active)
         {
            LogTools.info("Footstep Planning Request");

            startingStancePose.get(RobotSide.LEFT).setFromReferenceFrame(referenceFrames.getSoleFrame(RobotSide.LEFT));
            startingStancePose.get(RobotSide.RIGHT).setFromReferenceFrame(referenceFrames.getSoleFrame(RobotSide.RIGHT));
            goalStancePose.get(RobotSide.LEFT).setToZero();
            goalStancePose.get(RobotSide.RIGHT).setToZero();

            robotLocation.set((startingStancePose.get(RobotSide.LEFT).getX() + startingStancePose.get(RobotSide.RIGHT).getX()) / 2.0f,
                              (startingStancePose.get(RobotSide.LEFT).getY() + startingStancePose.get(RobotSide.RIGHT).getY()) / 2.0f);

            //Pose2D robotPose2D = new Pose2D(robotLocation.getX(), robotLocation.getY(), leftSolePose.getYaw());
            //ActiveMappingTools.getStraightGoalFootPoses(leftSolePose, rightSolePose, goalPose.get(RobotSide.LEFT, goalPose.get(RobotSide.RIGHT), 0.6f);
            //Pose2D goalPose2D = ActiveMappingTools.getNearestUnexploredNode(planarRegionMap.getMapRegions(), gridOrigin, robotPose2D, gridSize, gridResolution);

            monteCarloPathPlanner.getAgent().measure(monteCarloPathPlanner.getWorld());

            agentPositionIndices.set(monteCarloPathPlanner.getAgent().getState());

            robotLocationIndices.set(HeightMapTools.getIndexFromCoordinates(robotLocation.getX(), gridResolution, offset),
                                     HeightMapTools.getIndexFromCoordinates(robotLocation.getY(), gridResolution, offset));

            double error = agentPositionIndices.distance(robotLocationIndices);

            LogTools.info("Error: {}, Robot Position: {}, Agent Position: {}", error, robotLocationIndices, agentPositionIndices);

            if (error < 10.0f)
            {
               goalPositionIndices.set((Point2D) monteCarloPathPlanner.plan().getState());
               goalPosition.set(HeightMapTools.getCoordinateFromIndex((int) goalPositionIndices.getX(), gridResolution, offset),
                                HeightMapTools.getCoordinateFromIndex((int) goalPositionIndices.getY(), gridResolution, offset));

               monteCarloPathPlanner.updateState(new MonteCarloWaypointNode(goalPositionIndices, null, 0));
            }

            float yawRobotToGoal = (float) Math.atan2(goalPosition.getY() - robotLocation.getY(), goalPosition.getX() - robotLocation.getX());

            LogTools.info("Next Goal: Position:{}, Yaw{}", goalPosition, yawRobotToGoal);

            Pose3D goalPoseToUse = new Pose3D(goalPosition.getX(), goalPosition.getY(), 0.0, 0.0, 0.0, yawRobotToGoal);
            goalStancePose.get(RobotSide.LEFT).set(goalPoseToUse);
            goalStancePose.get(RobotSide.RIGHT).set(goalPoseToUse);
            goalStancePose.get(RobotSide.LEFT).prependTranslation(0.0, 0.12, 0.0);
            goalStancePose.get(RobotSide.RIGHT).prependTranslation(0.0, -0.12, 0.0);

            FootstepPlannerRequest request = createFootstepPlannerRequest(startingStancePose, goalStancePose);
            request.setPlanarRegionsList(planarRegionMap.getMapRegions());
            FootstepPlannerOutput plannerOutput = footstepPlanner.handleRequest(request);

            if (plannerOutput != null)
            {
               FootstepPlanningResult footstepPlanningResult = plannerOutput.getFootstepPlanningResult();
               LogTools.info("Footstep Planning Result: {}", footstepPlanningResult);
               LogTools.info(String.format("Planar Regions: %d\t, First Area: %.2f\t Plan Length: %d\n",
                                           planarRegionMap.getMapRegions().getNumberOfPlanarRegions(),
                                           planarRegionMap.getMapRegions().getPlanarRegion(0).getArea(),
                                           footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps()));

               planAvailable = footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps() > 0;
            }
         }
      }
   }

   public FootstepPlannerRequest createFootstepPlannerRequest(SideDependentList<FramePose3D> startPose, SideDependentList<FramePose3D> goalPose)
   {
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setStartFootPoses(startPose.get(RobotSide.LEFT), startPose.get(RobotSide.RIGHT));
      request.setGoalFootPoses(goalPose.get(RobotSide.LEFT), goalPose.get(RobotSide.RIGHT));
      request.setSwingPlannerType(SwingPlannerType.MULTI_WAYPOINT_POSITION);
      request.setPerformAStarSearch(true);
      request.setAssumeFlatGround(false);
      request.setPlanBodyPath(false);
      return request;
   }

   public FootstepDataListMessage getLimitedFootstepDataListMessage(ContinuousWalkingParameters parameters, List<QueuedFootstepStatusMessage> controllerQueue)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(parameters.getSwingTime());
      footstepDataListMessage.setDefaultTransferDuration(parameters.getTransferTime());

      // We expect the plannerOutput to contain this number of steps we ask for
      int index = 0;
      if (!controllerQueue.isEmpty() && !continuousPlanningParameters.getOverrideEntireQueueEachStep())
      {
         PlannedFootstep stepToNotOverride = new PlannedFootstep(RobotSide.fromByte(controllerQueue.get(1).getRobotSide()),
                                                                 new Pose3D(controllerQueue.get(1).getLocation(), controllerQueue.get(1).getOrientation()));
         footstepDataListMessage.getFootstepDataList().add().set(stepToNotOverride.getAsMessage());
         index = 1;
      }

      int totalNumberOfSteps = Math.min(latestFootstepPlan.getNumberOfSteps(), parameters.getNumberOfStepsToSend());
      for (int i = index; i < totalNumberOfSteps; i++)
      {
         PlannedFootstep footstep = latestFootstepPlan.getFootstep(i);
         footstep.limitFootholdVertices();
         footstepDataListMessage.getFootstepDataList().add().set(footstep.getAsMessage());
      }

      return footstepDataListMessage;
   }

   public FootstepDataListMessage getFootstepDataListMessage()
   {
      LogTools.info("Sending Plan to Controller: {}", latestFootstepPlan);
      return FootstepDataMessageConverter.createFootstepDataListFromPlan(latestFootstepPlan, 2.0, 1.0);
   }

   /**
    * This method gets called when we need a new footstep plan, this gets the latest information in the controller footstep queue
    */
   public void getImminentStanceFromLatestStatus(AtomicReference<FootstepStatusMessage> footstepStatusMessage,
                                                 List<QueuedFootstepStatusMessage> controllerQueue)
   {
      RobotSide imminentFootstepSide = RobotSide.fromByte(footstepStatusMessage.get().getRobotSide());
      assert imminentFootstepSide != null;

      int index = getNextIndexOnOppositeSide(RobotSide.fromByte(footstepStatusMessage.get().getRobotSide()), controllerQueue);

      FramePose3D imminentFootstepPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                         footstepStatusMessage.get().getDesiredFootPositionInWorld(),
                                                         footstepStatusMessage.get().getDesiredFootOrientationInWorld());

      FramePose3D nextRobotStepAfterCurrent;

      if (continuousPlanningParameters.getOverrideEntireQueueEachStep())
      {
         nextRobotStepAfterCurrent = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     referenceFrames.getSoleFrame(imminentFootstepSide.getOppositeSide()).getTransformToWorldFrame());
      }
      else
      {
         nextRobotStepAfterCurrent = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     controllerQueue.get(index).getLocation(),
                                                     controllerQueue.get(index).getOrientation());
      }

      updateImminentStance(nextRobotStepAfterCurrent, imminentFootstepPose, imminentFootstepSide);
   }

   private int getNextIndexOnOppositeSide(RobotSide side, List<QueuedFootstepStatusMessage> controllerQueue)
   {
      int i = 0;
      while (i < controllerQueue.size() && RobotSide.fromByte(controllerQueue.get(i).getRobotSide()) == side)
         i++;

      return i;
   }

   public void updateImminentStance(FramePose3D nextRobotStepAfterCurrent, FramePose3D imminentFootstepPose, RobotSide imminentFootstepSide)
   {
      this.imminentFootstepPose.set(imminentFootstepPose);
      this.imminentFootstepSide = imminentFootstepSide;
      startingStancePose.get(imminentFootstepSide.getOppositeSide()).set(nextRobotStepAfterCurrent);
      startingStancePose.get(imminentFootstepSide).set(imminentFootstepPose);
   }

   public void transitionCallback()
   {
      switch (mode)
      {
         case WALK_TO_GOAL -> {
            LogTools.info("Setting Previously Sent Plan for Reference");
            this.previousFootstepPlan = new FootstepPlan(latestFootstepPlan);
         }
         case INCREMENTAL_PLANNING -> monteCarloFootstepPlanner.transitionToOptimal();
      }
   }

   public FootstepPlanningResult getFootstepPlanningResult()
   {
      return footstepPlanningResult;
   }

   public FootstepPlan getLatestFootstepPlan()
   {
      return latestFootstepPlan;
   }

   public List<EnumMap<Axis3D, List<PolynomialReadOnly>>> getLatestSwingTrajectories()
   {
      return latestSwingTrajectories;
   }

   public void setPlanAvailable(boolean planAvailable)
   {
      this.planAvailable = planAvailable;
   }

   public boolean isPlanAvailable()
   {
      return planAvailable;
   }

   public boolean isActive()
   {
      return active;
   }

   public void setActive(boolean active)
   {
      this.active = active;
   }

   public void setInitialized(boolean initialized)
   {
      this.initialized = initialized;
   }

   public boolean isInitialized()
   {
      return initialized;
   }

   public Point2D getGridOrigin()
   {
      return gridOrigin;
   }

   public float getGridResolution()
   {
      return gridResolution;
   }

   public int getGridSize()
   {
      return monteCarloPathPlanner.getWorld().getGridHeight();
   }

   public void submitRangeScan(List<Point3DReadOnly> points)
   {
      monteCarloPathPlanner.submitMeasurements(points);
   }

   public MonteCarloPathPlanner getPlanner()
   {
      return monteCarloPathPlanner;
   }

   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return footstepPlanner.getFootstepPlannerParameters();
   }

   public MonteCarloFootstepPlannerParameters getMonteCarloFootstepPlannerParameters()
   {
      return monteCarloFootstepPlannerParameters;
   }

   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return swingPlannerParameters;
   }

   public SideDependentList<FramePose3D> getGoalStancePose()
   {
      return goalStancePose;
   }

   public SideDependentList<FramePose3D> getStartingStancePose()
   {
      return startingStancePose;
   }

   public PlanningMode getMode()
   {
      return mode;
   }
}
