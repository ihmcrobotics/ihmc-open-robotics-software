package us.ihmc.footstepPlanning.roughTerrainPlanning;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.graphSearch.SimplePlanarRegionBipedalAnytimeFootstepPlanner;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public class AnytimeFootstepPlannerOnRoughTerrainTest implements PlanningTest
{
   private static final boolean visualize = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 300000)
   public void testOverCinderBlockFieldWithGoalTooFarAway()
   {
      double startX = 0.0;
      double startY = 0.0;
      double cinderBlockSize = 0.4;
      int courseWidthXInNumberOfBlocks = 21;
      int courseLengthYInNumberOfBlocks = 6;

      PlanarRegionsList cinderBlockField = PlanarRegionsListExamples.generateCinderBlockField(startX, startY, cinderBlockSize, courseWidthXInNumberOfBlocks, courseLengthYInNumberOfBlocks);

      FramePose initialStanceFootPose = new FramePose(worldFrame);
      initialStanceFootPose.setPosition(0.0, -0.7, 0.0);
      RobotSide initialStanceSide = RobotSide.RIGHT;

      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(12.0, 0.7, 0.0);

      SideDependentList<FramePose> goalPoses = new SideDependentList<>();

      for(RobotSide robotSide : RobotSide.values)
      {
         FramePose footstepGoalPose = new FramePose(goalPose);
         footstepGoalPose.translate(0.0, robotSide.negateIfRightSide(0.2), 0.0);
         goalPoses.put(robotSide, footstepGoalPose);
      }

      SimplePlanarRegionBipedalAnytimeFootstepPlanner anytimePlanner = getPlanner();
      Runnable runnable = PlanningTestTools.createAnytimePlannerRunnable(anytimePlanner, initialStanceFootPose, initialStanceSide, goalPose, cinderBlockField);

      new Thread(runnable).start();
      checkThatPlannerKeepsGettingCloserToGoal(anytimePlanner, goalPoses);
      anytimePlanner.requestStop();

      FootstepPlan bestPlan = anytimePlanner.getBestPlanYet();
      double expectedClosestDistanceToGoal = 3.0;
      double actualClosestDistanceToGoal = getDistanceFromPlansLastFootstepToGoalFootstep(bestPlan, goalPoses);
//      assertTrue(actualClosestDistanceToGoal < expectedClosestDistanceToGoal);

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(cinderBlockField, bestPlan, goalPose);
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 300000)
   public void testOverCinderBlockFieldWithIncrementalKnowledgeOfTerrain()
   {
      double cinderBlockSize = 0.4;
      int courseWidthXInNumberOfBlocks = 7;
      int courseLengthYInNumberOfBlocks = 6;

      PlanarRegionsList cinderBlockFieldSection0 = PlanarRegionsListExamples.generateCinderBlockField(0.0, 0.0, cinderBlockSize, courseWidthXInNumberOfBlocks, courseLengthYInNumberOfBlocks);
      PlanarRegionsList cinderBlockFieldSection1 = PlanarRegionsListExamples.generateCinderBlockField(cinderBlockSize * courseWidthXInNumberOfBlocks, 0.0, cinderBlockSize, courseWidthXInNumberOfBlocks, courseLengthYInNumberOfBlocks);
      PlanarRegionsList cinderBlockFieldSection2 = PlanarRegionsListExamples.generateCinderBlockField(cinderBlockSize * courseWidthXInNumberOfBlocks, 0.0, cinderBlockSize, courseWidthXInNumberOfBlocks, courseLengthYInNumberOfBlocks);

      FramePose initialStanceFootPose = new FramePose(worldFrame);
      initialStanceFootPose.setPosition(0.0, -0.7, 0.0);
      RobotSide initialStanceSide = RobotSide.RIGHT;

      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(12.0, 0.7, 0.0);

      SideDependentList<FramePose> goalPoses = new SideDependentList<>();

      for(RobotSide robotSide : RobotSide.values)
      {
         FramePose footstepGoalPose = new FramePose(goalPose);
         footstepGoalPose.translate(0.0, robotSide.negateIfRightSide(0.2), 0.0);
         goalPoses.put(robotSide, footstepGoalPose);
      }

      SimplePlanarRegionBipedalAnytimeFootstepPlanner anytimePlanner = getPlanner();
      Runnable runnable = PlanningTestTools.createAnytimePlannerRunnable(anytimePlanner, initialStanceFootPose, initialStanceSide, goalPose, cinderBlockFieldSection0);

      new Thread(runnable).start();
      checkThatPlannerKeepsGettingCloserToGoal(anytimePlanner, goalPoses);
      anytimePlanner.requestStop();

      FootstepPlan bestPlan = anytimePlanner.getBestPlanYet();
      double expectedClosestDistanceToGoal = 3.0;
      double actualClosestDistanceToGoal = getDistanceFromPlansLastFootstepToGoalFootstep(bestPlan, goalPoses);
      assertTrue(actualClosestDistanceToGoal < expectedClosestDistanceToGoal);

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(cinderBlockFieldSection0, anytimePlanner.getBestPlanYet(), goalPose);
      }
   }

   private void checkThatPlannerKeepsGettingCloserToGoal(SimplePlanarRegionBipedalAnytimeFootstepPlanner anytimePlanner, SideDependentList<FramePose> goalPoses)
   {
      double closestFootstepToGoal = Double.MAX_VALUE;

      for(int i = 0; i < 100; i++)
      {
         ThreadTools.sleep(50);

         FootstepPlan bestPlanYet = anytimePlanner.getBestPlanYet();
         System.out.println("iteration " + i + ", num footsteps " + bestPlanYet.getNumberOfSteps());

         int numberOfFootsteps = bestPlanYet.getNumberOfSteps();
         if(numberOfFootsteps < 2)
            continue;

         double newClosestDistance = getDistanceFromPlansLastFootstepToGoalFootstep(bestPlanYet, goalPoses);

         assertTrue(newClosestDistance <= closestFootstepToGoal);
         closestFootstepToGoal = newClosestDistance;
      }
   }

   private double getDistanceFromPlansLastFootstepToGoalFootstep(FootstepPlan footstepPlan, SideDependentList<FramePose> goalPoses)
   {
      FramePose lastFootstepPose = new FramePose();
      int numberOfFootsteps = footstepPlan.getNumberOfSteps();
      if(numberOfFootsteps < 2)
         return Double.MAX_VALUE;

      SimpleFootstep lastFootstep = footstepPlan.getFootstep(numberOfFootsteps - 1);
      lastFootstep.getSoleFramePose(lastFootstepPose);
      FramePose footstepGoalPose = goalPoses.get(lastFootstep.getRobotSide());
      return lastFootstepPose.getPositionDistance(footstepGoalPose);
   }

   @Override
   public SimplePlanarRegionBipedalAnytimeFootstepPlanner getPlanner()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      SimplePlanarRegionBipedalAnytimeFootstepPlanner planner = new SimplePlanarRegionBipedalAnytimeFootstepPlanner(registry);

      planner.setMaximumStepReach(0.45);
      planner.setMaximumStepZ(0.25);
      planner.setMaximumStepYaw(0.15);
      planner.setMinimumStepWidth(0.15);
      planner.setMinimumFootholdPercent(0.8);

      double idealFootstepLength = 0.3;
      double idealFootstepWidth = 0.2;
      planner.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      if (visualize)
      {
         PlanarRegionBipedalFootstepPlannerVisualizer visualizer = SCSPlanarRegionBipedalFootstepPlannerVisualizer.createWithSimulationConstructionSet(1.0, footPolygonsInSoleFrame);
         planner.setBipedalFootstepPlannerListener(visualizer);
      }

      planner.setMaximumNumberOfNodesToExpand(100);

      return planner;
   }

   @Override
   public boolean visualize()
   {
      return visualize;
   }
}
