package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlanner;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class ActiveMappingModule
{
   public enum ActiveMappingMode
   {
      EXECUTE_AND_PAUSE, CONTINUOUS_MAPPING_STRAIGHT, CONTINUOUS_MAPPING_COVERAGE, CONTINUOUS_MAPPING_SEARCH
   }

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
   private Point2D robotLocation = new Point2D();

   private Pose3D leftGoalPose = new Pose3D();
   private Pose3D rightGoalPose = new Pose3D();
   private Pose3D leftSolePose = new Pose3D();
   private Pose3D rightSolePose = new Pose3D();

   private ArrayList<Point2D> frontierPoints = new ArrayList<>();

   private WalkingStatus walkingStatus = WalkingStatus.STARTED;

   private boolean planAvailable = false;
   private boolean active = true;

   private int gridSize = 20;
   private float gridResolution = 1.0f;

   int goalMargin = 5;
   int scale = 2;
   int worldHeight = 100;
   int worldWidth = 100;

   int iterations = 10;
   int simulationCount = 10;

   private final Point2D agentPos = new Point2D(10, 10);
   private final Point2D goal = new Point2D(10, worldHeight - 10);

   public ActiveMappingModule(DRCRobotModel robotModel, HumanoidReferenceFrames humanoidReferenceFrames)
   {
      this.referenceFrames = humanoidReferenceFrames;
      this.robotModel = robotModel;

      monteCarloPlanner = new MonteCarloPlanner();

      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);

      active = true;
   }

   public void updatePlan(PlanarRegionMap planarRegionMap)
   {
      if (active)
      {
         LogTools.info("Footstep Planning Request");

         leftSolePose.set(referenceFrames.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame());
         rightSolePose.set(referenceFrames.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());
         leftGoalPose.setToZero();
         rightGoalPose.setToZero();

         robotLocation.set((leftSolePose.getX() + rightSolePose.getX()) / 2.0f, (leftSolePose.getY() + rightSolePose.getY()) / 2.0f);

         Pose2D robotPose2D = new Pose2D(robotLocation.getX(), robotLocation.getY(), leftSolePose.getYaw());

         //ActiveMappingTools.getStraightGoalFootPoses(leftSolePose, rightSolePose, leftGoalPose, rightGoalPose, 0.6f);
         //Pose2D goalPose2D = ActiveMappingTools.getNearestUnexploredNode(planarRegionMap.getMapRegions(), gridOrigin, robotPose2D, gridSize, gridResolution);

         Mat gridColor = new Mat();
         MonteCarloPlannerTools.plotWorld(monteCarloPlanner.getWorld(), gridColor);
         PerceptionDebugTools.display("Grid", gridColor, 50, 1400);
         Point2D goalPosition = monteCarloPlanner.plan();
         monteCarloPlanner.execute(goalPosition);

         float yawRobotToGoal = (float) Math.atan2(goalPosition.getY() - robotLocation.getY(), goalPosition.getX() - robotLocation.getX());

         Pose3D goalPose = new Pose3D(goalPosition.getX(), goalPosition.getY(), 0.0, 0.0, 0.0, yawRobotToGoal);

         leftGoalPose.set(goalPose);
         rightGoalPose.set(goalPose);
         leftGoalPose.prependTranslation(0.0, 0.12, 0.0);
         rightGoalPose.prependTranslation(0.0, -0.12, 0.0);

         LogTools.info("Next Goal: {}", goalPose);

         request = new FootstepPlannerRequest();
         request.setTimeout(0.3);
         request.setStartFootPoses(leftSolePose, rightSolePose);
         request.setPlanarRegionsList(planarRegionMap.getMapRegions());
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
      return FootstepDataMessageConverter.createFootstepDataListFromPlan(plannerOutput.getFootstepPlan(), 1.3, 0.4);
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
      return gridSize;
   }

   public void submitRangeScan(ArrayList<Point3D> points)
   {
      monteCarloPlanner.addMeasurements(points);
   }

   public MonteCarloPlanner getPlanner()
   {
      return monteCarloPlanner;
   }
}
