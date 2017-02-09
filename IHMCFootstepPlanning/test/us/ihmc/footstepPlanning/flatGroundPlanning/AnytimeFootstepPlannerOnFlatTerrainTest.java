package us.ihmc.footstepPlanning.flatGroundPlanning;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.BipedalFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.PlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.graphSearch.SimplePlanarRegionBipedalAnytimeFootstepPlanner;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.footstepPlanning.roughTerrainPlanning.SCSPlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT})
public class AnytimeFootstepPlannerOnFlatTerrainTest implements PlanningTest
{
   private static final boolean visualize = false;
   private boolean assertPlannerReturnedResult = true;

   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testJustStraightLine()
   {
      PlanarRegionsList flatTerrain = PlanarRegionsListExamples.generateFlatGround(20.0, 20.0);

      FramePose initialStanceFootPose = new FramePose(worldFrame);
      initialStanceFootPose.setPosition(0.0, -0.7, 0.0);
      RobotSide initialStanceSide = RobotSide.RIGHT;

      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(15.0, 0.7, 0.0);

      SideDependentList<FramePose> goalPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose footstepGoalPose = new FramePose(goalPose);
         footstepGoalPose.translate(0.0, robotSide.negateIfRightSide(0.2), 0.0);
         goalPoses.put(robotSide, footstepGoalPose);
      }

      SimplePlanarRegionBipedalAnytimeFootstepPlanner anytimePlanner = getPlanner();
      PlanningTestTools.configureAnytimePlannerRunnable(anytimePlanner, initialStanceFootPose, initialStanceSide, goalPose, flatTerrain);

      new Thread(anytimePlanner).start();

      ThreadTools.sleep(100);

      double closestFootstepToGoal = Double.MAX_VALUE;
      int numIterations = 10;
      FootstepPlan bestPlanYet = anytimePlanner.getBestPlanYet();
      FramePose lastFootstepPose = new FramePose();

      for (int i = 0; i < numIterations; i++)
      {
         ThreadTools.sleep(100);
         FootstepPlan newPlan = anytimePlanner.getBestPlanYet();

         if (newPlan == null || newPlan.getNumberOfSteps() < 2)
            continue;

         bestPlanYet = newPlan;
         SimpleFootstep lastFootstep = bestPlanYet.getFootstep(bestPlanYet.getNumberOfSteps() - 1);
         lastFootstep.getSoleFramePose(lastFootstepPose);
         FramePose footstepGoalPose = goalPoses.get(lastFootstep.getRobotSide());

         double newClosestDistance = lastFootstepPose.getPositionDistance(footstepGoalPose);

         if (assertPlannerReturnedResult) assertTrue("newClosestDistance = " + newClosestDistance + ", closestFootstepToGoal = " + closestFootstepToGoal, newClosestDistance <= closestFootstepToGoal);
         closestFootstepToGoal = newClosestDistance;
      }

      double expectedClosestDistanceToGoal = 5.5;
      if (assertPlannerReturnedResult) assertTrue(closestFootstepToGoal < expectedClosestDistanceToGoal);

      ThreadTools.sleep(100);
      anytimePlanner.requestStop();

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(flatTerrain, bestPlanYet, goalPose);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testPlannerAdjustsPlanAfterExecutingFootstep()
   {
      PlanarRegionsList flatTerrain = PlanarRegionsListExamples.generateFlatGround(20.0, 20.0);

      FramePose initialStanceFootPose = new FramePose(worldFrame);
      initialStanceFootPose.setPosition(0.0, -0.7, 0.0);
      RobotSide initialStanceSide = RobotSide.RIGHT;

      FramePose goalPose = new FramePose(worldFrame);
      goalPose.setPosition(7.0, 0.7, 0.0);

      SideDependentList<FramePose> goalPoses = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose footstepGoalPose = new FramePose(goalPose);
         footstepGoalPose.translate(0.0, robotSide.negateIfRightSide(0.2), 0.0);
         goalPoses.put(robotSide, footstepGoalPose);
      }

      SimplePlanarRegionBipedalAnytimeFootstepPlanner anytimePlanner = getPlanner();
      PlanningTestTools.configureAnytimePlannerRunnable(anytimePlanner, initialStanceFootPose, initialStanceSide, goalPose, flatTerrain);

      new Thread(anytimePlanner).start();
      ThreadTools.sleep(500);

      FootstepPlan bestPlanYet = anytimePlanner.getBestPlanYet();
      assertTrue(bestPlanYet.getNumberOfSteps() > 3);

      SimpleFootstep firstFootstepFirstPlan = bestPlanYet.getFootstep(1);
      SimpleFootstep secondFootstepFirstPlan = bestPlanYet.getFootstep(2);

      anytimePlanner.executingFootstep(firstFootstepFirstPlan);
      anytimePlanner.executingFootstep(secondFootstepFirstPlan);

      ThreadTools.sleep(500);
      bestPlanYet = anytimePlanner.getBestPlanYet();
      assertTrue(bestPlanYet.getNumberOfSteps() > 2);
      anytimePlanner.requestStop();

      SimpleFootstep initialFootstepSecondPlan = bestPlanYet.getFootstep(0);
      assertTrue(secondFootstepFirstPlan.epsilonEquals(initialFootstepSecondPlan, 1e-12));
   }

   @Override
   public SimplePlanarRegionBipedalAnytimeFootstepPlanner getPlanner()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      BipedalFootstepPlannerParameters parameters = new BipedalFootstepPlannerParameters(registry);

      parameters.setMaximumStepReach(0.4);
      parameters.setMaximumStepZ(0.25);
      parameters.setMaximumStepXWhenForwardAndDown(0.25);
      parameters.setMaximumStepZWhenForwardAndDown(0.25);
      parameters.setMaximumStepYaw(0.25);
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
         PlanarRegionBipedalFootstepPlannerVisualizer visualizer = SCSPlanarRegionBipedalFootstepPlannerVisualizer
               .createWithSimulationConstructionSet(1.0, footPolygonsInSoleFrame, registry);
         planner.setBipedalFootstepPlannerListener(visualizer);
      }

      planner.setMaximumNumberOfNodesToExpand(1000);

      return planner;
   }

   @Override
   public boolean visualize()
   {
      return visualize;
   }
}
