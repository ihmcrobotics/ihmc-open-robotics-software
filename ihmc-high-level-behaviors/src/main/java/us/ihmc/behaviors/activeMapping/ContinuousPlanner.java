package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlanner;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.tools.ActiveMappingTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.List;

public class ContinuousPlanner
{
   public enum PlanningMode
   {
      EXECUTE_AND_PAUSE, FRONTIER_EXPANSION, ACTIVE_SEARCH, WALK_TO_GOAL, RANDOM_WALK
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

   private FootstepPlan previouslySentPlanForReference;
   private HumanoidReferenceFrames referenceFrames;
   private FootstepPlanningModule footstepPlanner;
   private MonteCarloPlanner monteCarloPlanner;
   private FootstepPlannerOutput plannerOutput;
   private FootstepPlanningResult footstepPlanningResult;

   private ContinuousPlanningParameters continuousPlanningParameters;
   private ContinuousPlannerStatistics statistics;

   private boolean initialized = false;
   private boolean planAvailable = false;
   private boolean active;

   private float gridResolution = 10.0f;
   private int offset = 70;

   public ContinuousPlanner(DRCRobotModel robotModel,
                            HumanoidReferenceFrames humanoidReferenceFrames,
                            ContinuousPlannerStatistics statistics,
                            PlanningMode mode)
   {
      this.referenceFrames = humanoidReferenceFrames;
      this.mode = mode;
      this.statistics = statistics;

      active = true;

      switch (mode)
      {
         case FRONTIER_EXPANSION:
            monteCarloPlanner = new MonteCarloPlanner(offset);
            break;
         case WALK_TO_GOAL, RANDOM_WALK:
            footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel, "ForContinuousWalking");
            break;
      }
   }

   public void initialize()
   {
      footstepPlanner.clearCustomTerminationConditions();
      footstepPlanner.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) -> pathSize
                                                                                                                  >= continuousPlanningParameters.getNumberOfStepsToSend());

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

   public void setGoalWaypointPoses(ContinuousPlanningParameters continuousPlanningParameters)
   {
      this.continuousPlanningParameters = continuousPlanningParameters;

      switch (this.mode)
      {
         case WALK_TO_GOAL:
            ActiveMappingTools.setRandomizedStraightGoalPoses(walkingStartMidPose,
                                                              startingStancePose,
                                                              goalStancePose,
                                                              (float) continuousPlanningParameters.getGoalPoseForwardDistance(),
                                                              (float) continuousPlanningParameters.getGoalPoseUpDistance());
            break;
         case RANDOM_WALK:
            goalGenerator.updateCurrentPosition(new Point3D(startingStancePose.get(RobotSide.LEFT).getPosition()));
            ActiveMappingTools.setRandomGoalWithinBounds(goalGenerator.getNextLocation(),
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

            monteCarloPlanner.getAgent().measure(monteCarloPlanner.getWorld());
            agentPositionIndices.set(monteCarloPlanner.getAgent().getState());
            robotLocationIndices.set(ActiveMappingTools.getIndexFromCoordinates(robotLocation.getX(), gridResolution, offset),
                                     ActiveMappingTools.getIndexFromCoordinates(robotLocation.getY(), gridResolution, offset));

            double error = agentPositionIndices.distance(robotLocationIndices);

            if (error < 10.0f)
            {
               goalPositionIndices.set(monteCarloPlanner.plan());
               goalPosition.set(ActiveMappingTools.getCoordinateFromIndex((int) goalPositionIndices.getX(), gridResolution, offset),
                                ActiveMappingTools.getCoordinateFromIndex((int) goalPositionIndices.getY(), gridResolution, offset));

               monteCarloPlanner.updateState(goalPositionIndices);
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

   public void planToGoalWithHeightMap(HeightMapData heightMapData)
   {
      long startTime = System.currentTimeMillis();
      if (footstepPlanner.isPlanning())
      {
         LogTools.warn("Footstep Planner is Busy!");
         return;
      }

      FootstepPlannerRequest request = createFootstepPlannerRequest(startingStancePose, goalStancePose);
      request.setRequestedInitialStanceSide(imminentFootstepSide);
      request.setSnapGoalSteps(true);
      request.setHeightMapData(heightMapData);
      request.setAbortIfGoalStepSnappingFails(true);

      request.setTimeout(continuousPlanningParameters.getInitialPlanningTimeout());
      if (plannerOutput != null)
      {
         LogTools.info("Setting Previous Plan as Reference in the Request");
         FootstepPlan previousFootstepPlan = plannerOutput.getFootstepPlan();

         if (previousFootstepPlan.getNumberOfSteps() < continuousPlanningParameters.getNumberOfStepsToSend())
         {
            LogTools.error("Previous Plan for Reference: Not Enough Steps: {}!", previousFootstepPlan.getNumberOfSteps());
         }
         else
         {
            if (imminentFootstepSide == previousFootstepPlan.getFootstep(0).getRobotSide())
            {
               previousFootstepPlan.remove(0);
            }
            request.setReferencePlan(previouslySentPlanForReference);
            request.setTimeout(continuousPlanningParameters.getPlanningWithReferenceTimeout());
         }
      }
      else
      {
         LogTools.warn("No Reference Plan Available");
      }

      plannerOutput = footstepPlanner.handleRequest(request);

      if (plannerOutput != null)
      {
         footstepPlanningResult = plannerOutput.getFootstepPlanningResult();
         planAvailable = footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps() > 0;

         LogTools.info(String.format("Plan Result: %s, Steps: %d, Result: %s, Initial Stance: %s",
                                     footstepPlanningResult,
                                     footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps(),
                                     planAvailable,
                                     imminentFootstepSide));
      }

      statistics.setLastPlanningTime((float) (System.currentTimeMillis() - startTime) / 1000.0f);
   }

   public FootstepPlannerOutput planToExploreWithHeightMap(HeightMapData heightMapData)
   {
      MonteCarloPlannerTools.plotWorld(monteCarloPlanner.getWorld(), gridColor);
      MonteCarloPlannerTools.plotAgent(monteCarloPlanner.getAgent(), gridColor);
      MonteCarloPlannerTools.plotRangeScan(monteCarloPlanner.getAgent().getScanPoints(), gridColor);

      PerceptionDebugTools.display("Monte Carlo Planner World", gridColor, 1, 1400);

      if (active)
      {
         LogTools.info("Footstep Planning Request");

         FootstepPlannerRequest request = createFootstepPlannerRequest(startingStancePose, goalStancePose);
         request.setHeightMapData(heightMapData);
         plannerOutput = footstepPlanner.handleRequest(request);

         if (plannerOutput != null)
         {
            FootstepPlanningResult footstepPlanningResult = plannerOutput.getFootstepPlanningResult();
            LogTools.info("Footstep Planning Result: {}", footstepPlanningResult);
            LogTools.info(String.format("Plan Length: %d\n", footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps()));
            planAvailable = footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps() > 0;
         }
      }
      return plannerOutput;
   }

   public void planToGoalWithPlanarRegionMap(PlanarRegionMap planarRegionMap)
   {
      if (mode == PlanningMode.FRONTIER_EXPANSION)
      {
         MonteCarloPlannerTools.plotWorld(monteCarloPlanner.getWorld(), gridColor);
         MonteCarloPlannerTools.plotAgent(monteCarloPlanner.getAgent(), gridColor);
         MonteCarloPlannerTools.plotRangeScan(monteCarloPlanner.getAgent().getScanPoints(), gridColor);

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

            monteCarloPlanner.getAgent().measure(monteCarloPlanner.getWorld());

            agentPositionIndices.set(monteCarloPlanner.getAgent().getState());

            robotLocationIndices.set(ActiveMappingTools.getIndexFromCoordinates(robotLocation.getX(), gridResolution, offset),
                                     ActiveMappingTools.getIndexFromCoordinates(robotLocation.getY(), gridResolution, offset));

            double error = agentPositionIndices.distance(robotLocationIndices);

            LogTools.info("Error: {}, Robot Position: {}, Agent Position: {}", error, robotLocationIndices, agentPositionIndices);

            if (error < 10.0f)
            {
               goalPositionIndices.set(monteCarloPlanner.plan());
               goalPosition.set(ActiveMappingTools.getCoordinateFromIndex((int) goalPositionIndices.getX(), gridResolution, offset),
                                ActiveMappingTools.getCoordinateFromIndex((int) goalPositionIndices.getY(), gridResolution, offset));

               monteCarloPlanner.updateState(goalPositionIndices);
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
            plannerOutput = footstepPlanner.handleRequest(request);

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

   public FootstepDataListMessage getLimitedFootstepDataListMessage(int count, float swingDuration, float transferDuration)
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(swingDuration);
      footstepDataListMessage.setDefaultTransferDuration(transferDuration);

      // The planner may time out before getting the recommended number of steps, this makes sure we take the smaller of the values
      if (count > plannerOutput.getFootstepPlan().getNumberOfSteps())
         count = plannerOutput.getFootstepPlan().getNumberOfSteps();

      for (int i = 0; i < count; i++)
      {
         PlannedFootstep footstep = plannerOutput.getFootstepPlan().getFootstep(i);
         footstep.limitFootholdVertices();
         footstepDataListMessage.getFootstepDataList().add().set(footstep.getAsMessage());
      }

      return footstepDataListMessage;
   }

   public FootstepDataListMessage getFootstepDataListMessage()
   {
      LogTools.info("Sending Plan to Controller: {}", plannerOutput.getFootstepPlan());
      return FootstepDataMessageConverter.createFootstepDataListFromPlan(plannerOutput.getFootstepPlan(), 2.0, 1.0);
   }

   public void updateImminentStance(FramePose3D realRobotStancePose, FramePose3D imminentFootstepPose, RobotSide imminentFootstepSide)
   {
      this.imminentFootstepPose.set(imminentFootstepPose);
      this.imminentFootstepSide = imminentFootstepSide;
      startingStancePose.get(imminentFootstepSide.getOppositeSide()).set(realRobotStancePose);
      startingStancePose.get(imminentFootstepSide).set(imminentFootstepPose);
   }

   public void setPreviouslySentPlanForReference()
   {
      LogTools.info("Setting Previously Sent Plan for Reference");
      this.previouslySentPlanForReference = plannerOutput.getFootstepPlan();
   }

   public FootstepPlanningResult getFootstepPlanningResult()
   {
      return footstepPlanningResult;
   }

   public FootstepPlannerOutput getPlannerOutput()
   {
      return plannerOutput;
   }

   public FootstepPlan getFootstepPlan()
   {
      return plannerOutput.getFootstepPlan();
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
      return monteCarloPlanner.getWorld().getGridHeight();
   }

   public void submitRangeScan(List<Point3DReadOnly> points)
   {
      monteCarloPlanner.submitMeasurements(points);
   }

   public MonteCarloPlanner getPlanner()
   {
      return monteCarloPlanner;
   }

   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return footstepPlanner.getFootstepPlannerParameters();
   }

   public SideDependentList<FramePose3D> getGoalStancePose()
   {
      return goalStancePose;
   }

   public SideDependentList<FramePose3D> getStartingStancePose()
   {
      return startingStancePose;
   }
}
