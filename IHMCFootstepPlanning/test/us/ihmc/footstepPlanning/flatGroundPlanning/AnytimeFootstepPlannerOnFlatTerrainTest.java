package us.ihmc.footstepPlanning.flatGroundPlanning;

import org.junit.Test;
import us.ihmc.footstepPlanning.AnytimeFootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.SimplePlanarRegionBipedalAnytimeFootstepPlanner;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.footstepPlanning.roughTerrainPlanning.SCSPlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.testTools.PlanningTest;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Point3d;
import java.util.concurrent.TimeUnit;

import static org.junit.Assert.assertTrue;

public class AnytimeFootstepPlannerOnFlatTerrainTest implements PlanningTest
{
   private static final boolean visualize = true;
   protected static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
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

      for(RobotSide robotSide : RobotSide.values)
      {
         FramePose footstepGoalPose = new FramePose(goalPose);
         footstepGoalPose.translate(0.0, robotSide.negateIfRightSide(0.2), 0.0);
         goalPoses.put(robotSide, footstepGoalPose);
      }

      SimplePlanarRegionBipedalAnytimeFootstepPlanner anytimePlanner = getPlanner();
      Runnable runnable = PlanningTestTools.createAnytimePlannerRunnable(anytimePlanner, initialStanceFootPose, initialStanceSide, goalPose, flatTerrain);

      new Thread(runnable).start();

      double closestFootstepToGoal = Double.MAX_VALUE;
      int numIterations = 100;
      FootstepPlan bestPlanYet = anytimePlanner.getBestPlanYet();
      FramePose lastFootstepPose = new FramePose();

      for(int i = 0; i < numIterations; i++)
      {
         ThreadTools.sleep(10);
         bestPlanYet = anytimePlanner.getBestPlanYet();

         int numberOfFootsteps = bestPlanYet.getNumberOfSteps();
         if(numberOfFootsteps < 2)
            continue;

         SimpleFootstep lastFootstep = bestPlanYet.getFootstep(numberOfFootsteps - 1);
         lastFootstep.getSoleFramePose(lastFootstepPose);
         FramePose footstepGoalPose = goalPoses.get(lastFootstep.getRobotSide());

         double newClosestDistance = lastFootstepPose.getPositionDistance(footstepGoalPose);

         assertTrue(newClosestDistance <= closestFootstepToGoal);
         closestFootstepToGoal = newClosestDistance;
      }

      double expectedClosestDistanceToGoal = 5.5;
      assertTrue(closestFootstepToGoal < expectedClosestDistanceToGoal);
      anytimePlanner.requestStop();

      if (visualize())
      {
         PlanningTestTools.visualizeAndSleep(flatTerrain, bestPlanYet, goalPose);
      }
   }

   @Override
   public SimplePlanarRegionBipedalAnytimeFootstepPlanner getPlanner()
   {
      SimplePlanarRegionBipedalAnytimeFootstepPlanner planner = new SimplePlanarRegionBipedalAnytimeFootstepPlanner();

      planner.setMaximumStepReach(0.4);
      planner.setMaximumStepZ(0.25);
      planner.setMaximumStepYaw(0.25);
      planner.setMinimumStepWidth(0.15);
      planner.setMinimumFootholdPercent(0.8);

      double idealFootstepLength = 0.3;
      double idealFootstepWidth = 0.2;
      planner.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      planner.setFeetPolygons(footPolygonsInSoleFrame);

      if (visualize)
      {
         SCSPlanarRegionBipedalFootstepPlannerVisualizer visualizer = new SCSPlanarRegionBipedalFootstepPlannerVisualizer(footPolygonsInSoleFrame);
         planner.setBipedalFootstepPlannerListener(visualizer);
      }

      planner.setMaximumNumberOfNodesToExpand(1000);

      return planner;
   }

   @Override
   public boolean visualize()
   {
      return true;
   }
}
