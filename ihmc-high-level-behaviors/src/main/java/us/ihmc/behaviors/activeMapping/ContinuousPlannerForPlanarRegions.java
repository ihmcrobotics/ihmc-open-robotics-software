package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPathPlanner;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloWaypointNode;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.mapping.PlanarRegionMap;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.perception.gpuHeightMap.HeightMapTools;

public class ContinuousPlannerForPlanarRegions
{
   private final SideDependentList<FramePose3D> startingStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> goalStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final Point2D robotLocationIndices = new Point2D();
   private final Point2D agentPositionIndices = new Point2D();
   private final Point2D goalPositionIndices = new Point2D();
   private final Point2D robotLocation = new Point2D();
   private final Point2D goalPosition = new Point2D();
   private final Mat gridColor = new Mat();

   private boolean planAvailable = false;
   private float gridResolution = 10.0f;
   private int offset = 70;

   private HumanoidReferenceFrames referenceFrames;
   private MonteCarloPathPlanner monteCarloPathPlanner;

   public ContinuousPlannerForPlanarRegions(HumanoidReferenceFrames referenceFrames)
   {
      this.referenceFrames = referenceFrames;
   }

   public void setGoalWaypointPoses(float nominalStanceWidth)
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
      goalStancePose.get(RobotSide.LEFT).prependTranslation(0.0, nominalStanceWidth, 0.0);
      goalStancePose.get(RobotSide.RIGHT).prependTranslation(0.0, -nominalStanceWidth, 0.0);
   }

   public void planBodyPathWithPlanarRegionMap(PlanarRegionMap planarRegionMap)
   {
      monteCarloPathPlanner = new MonteCarloPathPlanner(offset);

      MonteCarloPlannerTools.plotWorld(monteCarloPathPlanner.getWorld(), gridColor);
      MonteCarloPlannerTools.plotAgent(monteCarloPathPlanner.getAgent(), gridColor);
      MonteCarloPlannerTools.plotRangeScan(monteCarloPathPlanner.getAgent().getScanPoints(), gridColor);

      PerceptionDebugTools.display("Monte Carlo Planner World", gridColor, 1, 1400);

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

         planAvailable = true;
      }

      float yawRobotToGoal = (float) Math.atan2(goalPosition.getY() - robotLocation.getY(), goalPosition.getX() - robotLocation.getX());

      LogTools.info("Next Goal: Position:{}, Yaw{}", goalPosition, yawRobotToGoal);

      Pose3D goalPoseToUse = new Pose3D(goalPosition.getX(), goalPosition.getY(), 0.0, 0.0, 0.0, yawRobotToGoal);
      goalStancePose.get(RobotSide.LEFT).set(goalPoseToUse);
      goalStancePose.get(RobotSide.RIGHT).set(goalPoseToUse);
      goalStancePose.get(RobotSide.LEFT).prependTranslation(0.0, 0.12, 0.0);
      goalStancePose.get(RobotSide.RIGHT).prependTranslation(0.0, -0.12, 0.0);
   }

   public boolean isPlanAvailable()
   {
      return planAvailable;
   }

   public FootstepDataListMessage getFootstepDataListMessage()
   {
      return FootstepDataMessageConverter.createFootstepDataListFromPlan(new FootstepPlan(), 2.0, 1.0);
   }

   public void setPlanAvailable(boolean planAvailable)
   {
      this.planAvailable = planAvailable;
   }

   public MonteCarloPathPlanner getPlanner()
   {
      return monteCarloPathPlanner;
   }
}
