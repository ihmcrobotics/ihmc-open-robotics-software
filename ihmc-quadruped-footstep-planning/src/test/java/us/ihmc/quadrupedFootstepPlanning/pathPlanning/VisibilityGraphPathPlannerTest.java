package us.ihmc.quadrupedFootstepPlanning.pathPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerTargetType;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.QuadrupedFootstepPlannerStart;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import static us.ihmc.robotics.Assert.assertTrue;

public class VisibilityGraphPathPlannerTest
{
   @Test
   public void testStartingOffPlanarRegion()
   {
      YoVariableRegistry testRegistry = new YoVariableRegistry("testRegistry");
      VisibilityGraphPathPlanner planner = new VisibilityGraphPathPlanner(new DefaultVisibilityGraphParameters(), testRegistry);

      ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
      planarRegionPolygon.addVertex(-5.0, 5.0);
      planarRegionPolygon.addVertex(-5.0, -5.0);
      planarRegionPolygon.addVertex(5.0, -5.0);
      planarRegionPolygon.addVertex(5.0, 5.0);
      planarRegionPolygon.update();;

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(), planarRegionPolygon);
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegion);


      QuadrupedFootstepPlannerGoal goal = new QuadrupedFootstepPlannerGoal();
      goal.setGoalType(FootstepPlannerTargetType.POSE_BETWEEN_FEET);
      goal.setGoalPose(new FramePose3D());

      // try within the region
      FramePose3D startPose = new FramePose3D();
      startPose.getPosition().set(-4.0, 0.0, 0.0);

      planner.setPlanarRegionsList(planarRegionsList);
      planner.setInitialBodyPose(startPose);
      planner.setGoal(goal);

      planner.setTimeout(10.0);

      assertTrue(planner.planWaypoints().validForExecution());

      // try outisde the region
      startPose = new FramePose3D();
      startPose.getPosition().set(-6.0, 0.0, 0.0);

      planner.setPlanarRegionsList(planarRegionsList);
      planner.setInitialBodyPose(startPose);
      planner.setGoal(goal);

      planner.setTimeout(10.0);

      assertTrue(planner.planWaypoints().validForExecution());
   }
}
