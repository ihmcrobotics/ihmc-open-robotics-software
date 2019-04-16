package us.ihmc.footstepPlanning.graphSearch.aStar;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.FootstepPlanningTestTools;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AStarTurnInPlaceTest
{
   @Test
   public void testTurningInPlace()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      DefaultFootstepPlanningParameters parameters = new DefaultFootstepPlanningParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.addRectangle(5.0, 5.0);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      double timeout = 0.7;
      FramePose3D startPose = new FramePose3D();
      startPose.setOrientationYawPitchRoll(0.0 * Math.PI, 0.0, 0.0);
      startPose.appendTranslation(0.0, 0.5 * parameters.getIdealFootstepWidth(), 0.0);
      RobotSide initialStanceSide = RobotSide.LEFT;

      FramePose3D goalPose = new FramePose3D();
      goalPose.setOrientationYawPitchRoll(Math.PI, 0.0, 0.0);

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);

      AStarFootstepPlanner planner = AStarFootstepPlanner.createPlanner(parameters, null, footPolygons, expansion, registry);
      planner.setTimeout(timeout);
      planner.setInitialStanceFoot(startPose, initialStanceSide);
      planner.setGoal(goal);
      planner.setPlanarRegions(planarRegionsList);

      FootstepPlanningResult planningResult = planner.plan();
      Assertions.assertTrue(planningResult.validForExecution(), "Planner failed to find a 180 degree turn within " + timeout + " seconds");
   }
}