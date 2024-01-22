package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlanner;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.tools.ActiveMappingTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.util.ArrayList;
import java.util.List;

public class ActiveMapper
{
   public enum ActiveMappingMode
   {
      EXECUTE_AND_PAUSE, CONTINUOUS_MAPPING_STRAIGHT, CONTINUOUS_MAPPING_COVERAGE, CONTINUOUS_MAPPING_SEARCH
   }

   private final Mat gridColor = new Mat();

   private DecisionLayer decisionLayer;

   public ActiveMappingMode activeMappingMode = ActiveMappingMode.CONTINUOUS_MAPPING_STRAIGHT;

   private final FootstepPlanningModule footstepPlanner;
   private final DRCRobotModel robotModel;
   private final HumanoidReferenceFrames referenceFrames;
   private MonteCarloPlanner monteCarloPlanner;

   private FootstepPlannerRequest request;
   private FootstepPlannerOutput plannerOutput;

   private RobotSide initialStanceSide = RobotSide.LEFT;

   private Point2D gridOrigin = new Point2D(0.0, -1.0);

   private Pose3D leftGoalPose = new Pose3D();
   private Pose3D rightGoalPose = new Pose3D();
   private Pose3D leftSolePose = new Pose3D();
   private Pose3D rightSolePose = new Pose3D();

   private ArrayList<Point2D> frontierPoints = new ArrayList<>();

   private WalkingStatus walkingStatus = WalkingStatus.STARTED;

   private final Point2D goalPosition = new Point2D();
   private final Point2D goalPositionIndices = new Point2D();

   private final Point2D agentPosition = new Point2D();
   private final Point2D agentPositionIndices = new Point2D();

   private final Point2D robotLocation = new Point2D();
   private final Point2D robotLocationIndices = new Point2D();

   private boolean planAvailable = false;
   private boolean active = true;

   private float gridResolution = 10.0f;
   private int offset = 70;

   public ActiveMapper(DRCRobotModel robotModel, HumanoidReferenceFrames humanoidReferenceFrames)
   {
      this.referenceFrames = humanoidReferenceFrames;
      this.robotModel = robotModel;

      monteCarloPlanner = new MonteCarloPlanner(offset);

      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);

      active = true;
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

         leftSolePose.set(referenceFrames.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame());
         rightSolePose.set(referenceFrames.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());
         leftGoalPose.setToZero();
         rightGoalPose.setToZero();

         robotLocation.set((leftSolePose.getX() + rightSolePose.getX()) / 2.0f, (leftSolePose.getY() + rightSolePose.getY()) / 2.0f);

         //Pose2D robotPose2D = new Pose2D(robotLocation.getX(), robotLocation.getY(), leftSolePose.getYaw());
         //ActiveMappingTools.getStraightGoalFootPoses(leftSolePose, rightSolePose, leftGoalPose, rightGoalPose, 0.6f);
         //Pose2D goalPose2D = ActiveMappingTools.getNearestUnexploredNode(planarRegionMap.getMapRegions(), gridOrigin, robotPose2D, gridSize, gridResolution);

         monteCarloPlanner.getAgent().measure(monteCarloPlanner.getWorld());


         agentPositionIndices.set(monteCarloPlanner.getAgent().getPosition());

         robotLocationIndices.set(ActiveMappingTools.getIndexFromCoordinates(robotLocation.getX(), gridResolution, offset),
                                  ActiveMappingTools.getIndexFromCoordinates(robotLocation.getY(), gridResolution, offset));

         double error = agentPositionIndices.distance(robotLocationIndices);

         LogTools.warn("Error: {}, Robot Position: {}, Agent Position: {}", error, robotLocationIndices, agentPositionIndices);

         if (error < 10.0f)
         {
            goalPositionIndices.set(monteCarloPlanner.plan());
            goalPosition.set(ActiveMappingTools.getCoordinateFromIndex((int) goalPositionIndices.getX(), gridResolution, offset),
                             ActiveMappingTools.getCoordinateFromIndex((int) goalPositionIndices.getY(), gridResolution, offset));

            monteCarloPlanner.updateState(goalPositionIndices);
         }

         float yawRobotToGoal = (float) Math.atan2(goalPosition.getY() - robotLocation.getY(), goalPosition.getX() - robotLocation.getX());

         Pose3D goalPose = new Pose3D(goalPosition.getX(), goalPosition.getY(), 0.0, 0.0, 0.0, yawRobotToGoal);

         leftGoalPose.set(goalPose);
         rightGoalPose.set(goalPose);
         leftGoalPose.prependTranslation(0.0, 0.12, 0.0);
         rightGoalPose.prependTranslation(0.0, -0.12, 0.0);

         LogTools.info("Next Goal: {}", goalPose);

         request = new FootstepPlannerRequest();
         request.setTimeout(1.5);
         request.setStartFootPoses(leftSolePose, rightSolePose);
         request.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionMap.getMapRegions())));
         request.setPlanBodyPath(false);
         request.setGoalFootPoses(leftGoalPose, rightGoalPose);
         request.setPerformAStarSearch(true);

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

   public FootstepDataListMessage getFootstepDataListMessage()
   {
      return FootstepDataMessageConverter.createFootstepDataListFromPlan(plannerOutput.getFootstepPlan(), 0.6, 0.3);
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

   public void setWalkingStatus(WalkingStatus walkingStatus)
   {
      this.walkingStatus = walkingStatus;
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
}
