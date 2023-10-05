package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlanner;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.tools.ActiveMappingTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;

public class ContinuousPlanner
{
   public enum PlanningMode
   {
      EXECUTE_AND_PAUSE, WALK_STRAIGHT, MAX_COVERAGE, ACTIVE_SEARCH
   }

   private final Mat gridColor = new Mat();

   private DecisionLayer decisionLayer;

   private final FootstepPlannerLogger footstepPlannerLogger;
   private final FootstepPlanningModule footstepPlanner;
   private final HumanoidReferenceFrames referenceFrames;
   private MonteCarloPlanner monteCarloPlanner;

   private FootstepPlannerOutput plannerOutput;

   private Point2D gridOrigin = new Point2D(0.0, -1.0);

   private SideDependentList<FramePose3D> goalPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private SideDependentList<FramePose3D> startPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private final Point2D goalPosition = new Point2D();
   private final Point2D goalPositionIndices = new Point2D();

   private final Point2D agentPosition = new Point2D();
   private final Point2D agentPositionIndices = new Point2D();

   private final Point2D robotLocation = new Point2D();
   private final Point2D robotLocationIndices = new Point2D();

   private boolean initialized = false;
   private boolean planAvailable = false;
   private boolean active;

   private float gridResolution = 10.0f;
   private int offset = 70;

   public ContinuousPlanner(DRCRobotModel robotModel, HumanoidReferenceFrames humanoidReferenceFrames, PlanningMode mode)
   {
      this.referenceFrames = humanoidReferenceFrames;
      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);
      active = true;

      switch (mode)
      {
         case MAX_COVERAGE:
            monteCarloPlanner = new MonteCarloPlanner(offset);
            break;
      }
   }

   public void updatePlan(PlanarRegionMap planarRegionMap)
   {
      MonteCarloPlannerTools.plotWorld(monteCarloPlanner.getWorld(), gridColor);
      MonteCarloPlannerTools.plotAgent(monteCarloPlanner.getAgent(), gridColor);
      MonteCarloPlannerTools.plotRangeScan(monteCarloPlanner.getAgent().getScanPoints(), gridColor);

      PerceptionDebugTools.display("Monte Carlo Planner World", gridColor, 1, 1400);

      if (active)
      {
         LogTools.info("Footstep Planning Request");

         startPose.get(RobotSide.LEFT).setFromReferenceFrame(referenceFrames.getSoleFrame(RobotSide.LEFT));
         startPose.get(RobotSide.RIGHT).setFromReferenceFrame(referenceFrames.getSoleFrame(RobotSide.RIGHT));
         goalPose.get(RobotSide.LEFT).setToZero();
         goalPose.get(RobotSide.RIGHT).setToZero();

         robotLocation.set((startPose.get(RobotSide.LEFT).getX() + startPose.get(RobotSide.RIGHT).getX()) / 2.0f, (startPose.get(RobotSide.LEFT).getY() + startPose.get(RobotSide.RIGHT).getY()) / 2.0f);

         //Pose2D robotPose2D = new Pose2D(robotLocation.getX(), robotLocation.getY(), leftSolePose.getYaw());
         //ActiveMappingTools.getStraightGoalFootPoses(leftSolePose, rightSolePose, goalPose.get(RobotSide.LEFT, goalPose.get(RobotSide.RIGHT), 0.6f);
         //Pose2D goalPose2D = ActiveMappingTools.getNearestUnexploredNode(planarRegionMap.getMapRegions(), gridOrigin, robotPose2D, gridSize, gridResolution);

         monteCarloPlanner.getAgent().measure(monteCarloPlanner.getWorld());


         agentPositionIndices.set(monteCarloPlanner.getAgent().getPosition());

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
         goalPose.get(RobotSide.LEFT).set(goalPoseToUse);
         goalPose.get(RobotSide.RIGHT).set(goalPoseToUse);
         goalPose.get(RobotSide.LEFT).prependTranslation(0.0, 0.12, 0.0);
         goalPose.get(RobotSide.RIGHT).prependTranslation(0.0, -0.12, 0.0);

         FootstepPlannerRequest request = createFootstepPlannerRequest(startPose, goalPose);
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

   public FootstepPlannerOutput updatePlan(HeightMapData heightMapData, SideDependentList<FramePose3D> startPose, SideDependentList<FramePose3D> goalPose, RobotSide stanceSide)
   {
      if (footstepPlanner.isPlanning())
      {
         LogTools.warn("Footstep Planner is Busy!");
         return null;
      }

      FootstepPlannerRequest request = createFootstepPlannerRequest(startPose, goalPose);
      request.setRequestedInitialStanceSide(stanceSide);
      request.setSnapGoalSteps(true);
      request.setHeightMapData(heightMapData);
      request.setAbortIfGoalStepSnappingFails(true);

      FootstepPlannerOutput plannerOutput;
      plannerOutput = footstepPlanner.handleRequest(request);
      footstepPlannerLogger.logSession();
      FootstepPlannerLogger.deleteOldLogs();

      if (plannerOutput != null)
      {
         FootstepPlanningResult footstepPlanningResult = plannerOutput.getFootstepPlanningResult();
         planAvailable = footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps() > 0;

         LogTools.info(String.format("Plan Result: %s, Steps: %d, Result: %s, Initial Stance: %s", footstepPlanningResult,
                                     footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps(), planAvailable, stanceSide));
      }

      return plannerOutput;
   }

   public FootstepPlannerRequest createFootstepPlannerRequest(SideDependentList<FramePose3D> startPose, SideDependentList<FramePose3D> goalPose)
   {
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setTimeout(0.25);
      request.setStartFootPoses(startPose.get(RobotSide.LEFT), startPose.get(RobotSide.RIGHT));
      request.setGoalFootPoses(goalPose.get(RobotSide.LEFT), goalPose.get(RobotSide.RIGHT));
      request.setPlanBodyPath(false);
      request.setPerformAStarSearch(true);
      request.setAssumeFlatGround(false);
      request.setSwingPlannerType(SwingPlannerType.NONE);
      return request;
   }

   public FootstepDataListMessage getFootstepDataListMessage()
   {
      LogTools.info("Sending Plan to Controller: {}", plannerOutput.getFootstepPlan());
      return FootstepDataMessageConverter.createFootstepDataListFromPlan(plannerOutput.getFootstepPlan(), 2.0, 1.0);
   }

   public FootstepDataListMessage getLimitedFootstepDataListMessage(FootstepPlannerOutput plannerOutput, int count, float swingDuration, float transferDuration)
   {
      LogTools.info("Sending Plan to Controller: {}", plannerOutput.getFootstepPlan());
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(swingDuration);
      footstepDataListMessage.setDefaultTransferDuration(transferDuration);

      for (int i = 0; i < count; i++)
      {
         PlannedFootstep footstep = plannerOutput.getFootstepPlan().getFootstep(i);
         footstep.limitFootholdVertices();
         footstepDataListMessage.getFootstepDataList().add().set(footstep.getAsMessage());

         LogTools.info("Footstep Side: {}", footstep.getRobotSide());
      }

      return footstepDataListMessage;
   }

   public SideDependentList<FramePose3D> updateStanceAndSwitchSides(FramePose3D lastSentFootstepPose, RobotSide lastSentFootstepSide)
   {
      SideDependentList<FramePose3D> startPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());

      startPose.get(lastSentFootstepSide.getOppositeSide()).setFromReferenceFrame(referenceFrames.getSoleFrame(lastSentFootstepSide.getOppositeSide()));
      startPose.get(lastSentFootstepSide).set(lastSentFootstepPose);

      LogTools.info("New Stance for Planning: {}", lastSentFootstepSide);

      return startPose;
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
}
