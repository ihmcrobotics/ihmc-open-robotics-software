package us.ihmc.footstepPlanning.roughTerrainPlanning;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.graphSearch.SimplePlanarRegionBipedalAnytimeFootstepPlanner;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegion;
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
   private boolean assertPlannerReturnedResult = true;
   private static final boolean visualize = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 300000)
   public void testOverCinderBlockFieldWithGoalTooFarAway()
   {
      double startX = 0.0;
      double startY = 0.0;
      double cinderBlockHeight = 0.15;
      double cinderBlockSize = 0.4;
      int courseWidthXInNumberOfBlocks = 21;
      int courseLengthYInNumberOfBlocks = 6;
      double heightVariation = 0.1;

      PlanarRegionsList cinderBlockField = PlanarRegionsListExamples.generateCinderBlockField(startX, startY, cinderBlockSize, cinderBlockHeight, courseWidthXInNumberOfBlocks, courseLengthYInNumberOfBlocks, heightVariation);

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
      PlanningTestTools.configureAnytimePlannerRunnable(anytimePlanner, initialStanceFootPose, initialStanceSide, goalPose, cinderBlockField);

      new Thread(anytimePlanner).start();
      FootstepPlan bestPlan = checkThatPlannerKeepsGettingCloserToGoal(anytimePlanner, goalPoses);
      anytimePlanner.requestStop();

      double expectedClosestDistanceToGoal = 3.0;
      double actualClosestDistanceToGoal = getDistanceFromPlansLastFootstepToGoalFootstep(bestPlan, goalPoses);
      if (assertPlannerReturnedResult) assertTrue("expectedClosestDistanceToGoal: " + expectedClosestDistanceToGoal + ", actualClosestDistanceToGoal: " + actualClosestDistanceToGoal, actualClosestDistanceToGoal < expectedClosestDistanceToGoal);

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(cinderBlockField, bestPlan, goalPose);
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 300000)
   public void testOverCinderBlockFieldWithIncrementalKnowledgeOfTerrain()
   {
      double cinderBlockHeight = 0.15;
      double cinderBlockSize = 0.4;
      int courseLengthYInNumberOfBlocks = 6;
      double heightVariation = 0.1;

      PlanarRegionsList cinderBlockFieldOneThird = PlanarRegionsListExamples.generateCinderBlockField(0.0, 0.0, cinderBlockHeight, cinderBlockSize, 7, courseLengthYInNumberOfBlocks, heightVariation);
      PlanarRegionsList cinderBlockFieldTwoThirds = PlanarRegionsListExamples.generateCinderBlockField(0.0, 0.0, cinderBlockHeight, cinderBlockSize, 14, courseLengthYInNumberOfBlocks, heightVariation);
      PlanarRegionsList cinderBlockEntireField = PlanarRegionsListExamples.generateCinderBlockField(0.0, 0.0, cinderBlockHeight, cinderBlockSize, 21, courseLengthYInNumberOfBlocks, heightVariation);

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
      PlanningTestTools.configureAnytimePlannerRunnable(anytimePlanner, initialStanceFootPose, initialStanceSide, goalPose, cinderBlockFieldOneThird);

      new Thread(anytimePlanner).start();

      checkThatPlannerKeepsGettingCloserToGoal(anytimePlanner, goalPoses);
      anytimePlanner.setPlanarRegions(cinderBlockFieldTwoThirds);
      checkThatPlannerKeepsGettingCloserToGoal(anytimePlanner, goalPoses);
      anytimePlanner.setPlanarRegions(cinderBlockEntireField);
      FootstepPlan bestPlanYet = checkThatPlannerKeepsGettingCloserToGoal(anytimePlanner, goalPoses);

      anytimePlanner.requestStop();
      double closestDistanceOfFootstepToGoal = getDistanceFromPlansLastFootstepToGoalFootstep(bestPlanYet, goalPoses);
      double expectedClosestDistanceToGoal = 3.0;
      if (assertPlannerReturnedResult) assertTrue("closestDistanceOfFootstepToGoal = " + closestDistanceOfFootstepToGoal + ", expectedClosestDistanceToGoal = " + expectedClosestDistanceToGoal, closestDistanceOfFootstepToGoal < expectedClosestDistanceToGoal);

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(cinderBlockEntireField, bestPlanYet, goalPose);
      }
   }

   private void appendPlanarRegionLists(PlanarRegionsList listToAppendTo, PlanarRegionsList... listsToAppendFrom)
   {
      for(int listIndex = 0; listIndex < listsToAppendFrom.length; listIndex++)
      {
         PlanarRegionsList list = listsToAppendFrom[listIndex];
         for(int regionIndex = 0; regionIndex < list.getNumberOfPlanarRegions(); regionIndex++)
         {
            listToAppendTo.addPlanarRegion(list.getPlanarRegion(regionIndex));
         }
      }
   }

   private FootstepPlan checkThatPlannerKeepsGettingCloserToGoal(SimplePlanarRegionBipedalAnytimeFootstepPlanner anytimePlanner, SideDependentList<FramePose> goalPoses)
   {
      FootstepPlan bestPlanYet = null;
      double closestFootstepToGoal = Double.MAX_VALUE;

      for(int i = 0; i < 10; i++)
      {
         ThreadTools.sleep(500);

         FootstepPlan newPlan = anytimePlanner.getBestPlanYet();
         if(newPlan == null)
            continue;
         else
            bestPlanYet = newPlan;

         int numberOfFootsteps = bestPlanYet.getNumberOfSteps();
         if(numberOfFootsteps < 2)
            continue;

         double newClosestDistance = getDistanceFromPlansLastFootstepToGoalFootstep(bestPlanYet, goalPoses);

         if (assertPlannerReturnedResult) assertTrue(newClosestDistance <= closestFootstepToGoal);
         closestFootstepToGoal = newClosestDistance;
      }

      return bestPlanYet;
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
      BipedalFootstepPlannerParameters parameters = new BipedalFootstepPlannerParameters(registry);

      parameters.setMaximumStepReach(0.45);
      parameters.setMaximumStepZ(0.25);
      parameters.setMaximumStepXWhenForwardAndDown(0.25);
      parameters.setMaximumStepZWhenForwardAndDown(0.25);
      parameters.setMaximumStepYaw(0.15);
      parameters.setMaximumStepWidth(0.4);
      parameters.setMinimumStepWidth(0.15);
      parameters.setMinimumFootholdPercent(0.8);

      double idealFootstepLength = 0.3;
      double idealFootstepWidth = 0.2;
      parameters.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      SimplePlanarRegionBipedalAnytimeFootstepPlanner planner = new SimplePlanarRegionBipedalAnytimeFootstepPlanner(parameters, registry);
      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      if (visualize)
      {
         PlanarRegionBipedalFootstepPlannerVisualizer visualizer = SCSPlanarRegionBipedalFootstepPlannerVisualizer.createWithSimulationConstructionSet(1.0, footPolygonsInSoleFrame, registry);
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
