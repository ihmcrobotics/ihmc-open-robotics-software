package us.ihmc.footstepPlanning.graphSearch;

import static org.junit.Assert.*;

import javax.vecmath.Point2d;

import org.junit.Test;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.footstepPlanning.roughTerrainPlanning.SCSPlanarRegionBipedalFootstepPlannerVisualizer;
import us.ihmc.footstepPlanning.simplePlanners.FlatGroundPlanningUtils;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.tools.thread.ThreadTools;

public class SimplePlanarRegionBipedalAnytimeFootstepPlannerTest
{
   private final boolean visualize = false;

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testSameResultsAsNormalPlannerWhenUsedAsANormalPlanner()
   {
      YoVariableRegistry registryOne = new YoVariableRegistry("One");
      SimplePlanarRegionBipedalAnytimeFootstepPlanner anytimePlannerOne = new SimplePlanarRegionBipedalAnytimeFootstepPlanner(registryOne);
      BipedalFootstepPlannerParameters parametersOne = anytimePlannerOne.getParameters();
      setParameters(parametersOne);

      YoVariableRegistry registryTwo = new YoVariableRegistry("Two");
      PlanarRegionBipedalFootstepPlanner normalPlannerTwo = new PlanarRegionBipedalFootstepPlanner(registryTwo);
      normalPlannerTwo.setMaximumNumberOfNodesToExpand(1000);
      BipedalFootstepPlannerParameters parametersTwo = normalPlannerTwo.getParameters();
      setParameters(parametersTwo);

      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrameOne = PlanningTestTools.createDefaultFootPolygons();
      anytimePlannerOne.setFeetPolygons(footPolygonsInSoleFrameOne);

      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrameTwo = PlanningTestTools.createDefaultFootPolygons();
      normalPlannerTwo.setFeetPolygons(footPolygonsInSoleFrameTwo);

      PlanarRegionBipedalFootstepPlannerVisualizer visualizerOne = null;
      PlanarRegionBipedalFootstepPlannerVisualizer visualizerTwo = null;

      if (visualize)
      {
         visualizerOne = SCSPlanarRegionBipedalFootstepPlannerVisualizer.createWithSimulationConstructionSet(0.01, footPolygonsInSoleFrameOne);
         visualizerTwo = SCSPlanarRegionBipedalFootstepPlannerVisualizer.createWithSimulationConstructionSet(0.01, footPolygonsInSoleFrameTwo);
         anytimePlannerOne.setBipedalFootstepPlannerListener(visualizerOne);
         normalPlannerTwo.setBipedalFootstepPlannerListener(visualizerTwo);
      }

      double startX = 0.0;
      double startY = 0.0;
      double cinderBlockSize = 0.4;
      int courseWidthXInNumberOfBlocks = 21;
      int courseLengthYInNumberOfBlocks = 6;
      PlanarRegionsList planarRegionsListOne = PlanarRegionsListExamples.generateCinderBlockField(startX, startY, cinderBlockSize, courseWidthXInNumberOfBlocks,
                                                                                                  courseLengthYInNumberOfBlocks);
      anytimePlannerOne.setPlanarRegions(planarRegionsListOne);

      PlanarRegionsList planarRegionsListTwo = PlanarRegionsListExamples.generateCinderBlockField(startX, startY, cinderBlockSize, courseWidthXInNumberOfBlocks,
                                                                                                  courseLengthYInNumberOfBlocks);
      normalPlannerTwo.setPlanarRegions(planarRegionsListTwo);

      setGoal(anytimePlannerOne, RobotSide.LEFT);
      setGoal(normalPlannerTwo, RobotSide.LEFT);

      FootstepPlanningResult resultOne = anytimePlannerOne.plan();
      FootstepPlanningResult resultTwo = normalPlannerTwo.plan();

      assertTrue(resultOne == resultTwo);

      FootstepPlan planOne = anytimePlannerOne.getPlan();
      FootstepPlan planTwo = normalPlannerTwo.getPlan();

      assertEquals(planOne.getNumberOfSteps(), planTwo.getNumberOfSteps());

      for (int i = 0; i < planOne.getNumberOfSteps(); i++)
      {
         SimpleFootstep footstepOne = planOne.getFootstep(i);
         SimpleFootstep footstepTwo = planTwo.getFootstep(i);

         assertTrue(footstepOne.epsilonEquals(footstepTwo, 1e-5));
      }

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }

   private void setGoal(FootstepPlanner planner, RobotSide initialStanceSide)
   {
      double xGoal = 9.0;
      double yGoal = 0.3;
      double yawGoal = 0.2;
      Point2d goalPosition = new Point2d(xGoal, yGoal);
      FramePose2d goalPose2d = new FramePose2d(ReferenceFrame.getWorldFrame(), goalPosition, yawGoal);

      double xInitialStanceFoot = 0.0;
      double yInitialStanceFoot = 0.0;
      double yawInitial = 0.0;
      Point2d initialStanceFootPosition = new Point2d(xInitialStanceFoot, yInitialStanceFoot);
      FramePose2d initialStanceFootPose2d = new FramePose2d(ReferenceFrame.getWorldFrame(), initialStanceFootPosition, yawInitial);

      FramePose initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose2d);
      FramePose goalPose = FlatGroundPlanningUtils.poseFormPose2d(goalPose2d);

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);
      goal.setXYGoal(new Point2d(goalPose.getX(), goalPose.getY()), 0.5);

      planner.setInitialStanceFoot(initialStanceFootPose3d, initialStanceSide);
      planner.setGoal(goal);
   }

   private void setParameters(BipedalFootstepPlannerParameters parameters)
   {
      parameters.setMaximumStepReach(0.4);
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
   }
}
