package us.ihmc.footstepPlanning.bodyPath;

import java.util.ArrayList;
import java.util.List;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TestName;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FlatGroundFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.AlwaysValidNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.SimpleSideBasedExpansion;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.BodyPathCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlan;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FootstepPlanningWithBodyPathTest
{
   private static final boolean visualize = true;

   @Rule
   public TestName name = new TestName();

   @Test
   public void testWaypointPathOnFlat()
   {
      YoVariableRegistry registry = new YoVariableRegistry(name.getMethodName());
      FootstepPlannerParameters parameters = new DefaultFootstepPlanningParameters();
      double defaultStepWidth = parameters.getIdealFootstepWidth();

      double goalDistance = 5.0;
      FramePose initialStanceFootPose = new FramePose();
      RobotSide initialStanceFootSide = RobotSide.LEFT;
      initialStanceFootPose.setY(initialStanceFootSide.negateIfRightSide(defaultStepWidth / 2.0));
      FramePose goalPose = new FramePose();
      goalPose.setX(goalDistance);

      WaypointDefinedBodyPathPlan bodyPath = new WaypointDefinedBodyPathPlan();
      List<Point2D> waypoints = new ArrayList<>();
      waypoints.add(new Point2D(0.0, 0.0));
      waypoints.add(new Point2D(goalDistance / 3.0, 1.0));
      waypoints.add(new Point2D(2.0 * goalDistance / 3.0, -1.0));
      waypoints.add(new Point2D(goalDistance, 0.0));
      bodyPath.setWaypoints(waypoints);
      bodyPath.compute(null, null);

      FootstepNodeChecker nodeChecker = new AlwaysValidNodeChecker();
      CostToGoHeuristics heuristics = new BodyPathHeuristics(registry, parameters, bodyPath);
      FootstepNodeExpansion nodeExpansion = new SimpleSideBasedExpansion(parameters);
      FootstepCost stepCostCalculator = new BodyPathCost(parameters, bodyPath);
      FlatGroundFootstepNodeSnapper snapper = new FlatGroundFootstepNodeSnapper();

      heuristics.setWeight(1.3);
      FootstepPlanner planner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, nodeExpansion, stepCostCalculator, snapper, registry);

      FootstepPlan footstepPlan = PlanningTestTools.runPlanner(planner, initialStanceFootPose, initialStanceFootSide, goalPose, null, true);

      if (visualize)
         PlanningTestTools.visualizeAndSleep(null, footstepPlan, goalPose, bodyPath);
   }
}
