package us.ihmc.quadrupedFootstepPlanning.pathPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerGoal;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerTargetType;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import static us.ihmc.robotics.Assert.assertTrue;

public class VisibilityGraphPawPathPlannerTest
{
   @Test
   public void testStartingOffPlanarRegion()
   {
      YoVariableRegistry testRegistry = new YoVariableRegistry("testRegistry");
      VisibilityGraphPawPathPlanner planner = new VisibilityGraphPawPathPlanner(new DefaultVisibilityGraphParameters(), testRegistry);

      ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
      planarRegionPolygon.addVertex(-5.0, 5.0);
      planarRegionPolygon.addVertex(-5.0, -5.0);
      planarRegionPolygon.addVertex(5.0, -5.0);
      planarRegionPolygon.addVertex(5.0, 5.0);
      planarRegionPolygon.update();;

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(), planarRegionPolygon);
      planarRegion.setRegionId(25);
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegion);


      PawStepPlannerGoal goal = new PawStepPlannerGoal();
      goal.setGoalType(PawStepPlannerTargetType.POSE_BETWEEN_FEET);
      goal.setGoalPose(new FramePose3D());

      // try within the region
      FramePose3D startPose = new FramePose3D();
      startPose.getPosition().set(-4.0, 0.0, 0.0);

      planner.setPlanarRegionsList(planarRegionsList);
      planner.setInitialBodyPose(startPose);
      planner.setGoal(goal);

      planner.setTimeout(10.0);

//      assertTrue(planner.planWaypoints().validForExecution());

      // try outisde the region
      startPose = new FramePose3D();
      startPose.getPosition().set(-5.4, 0.0, 0.0); // default "size" is to add a 0.3 m support polygon, so this should leave a 0.1 m gap

      planner.setPlanarRegionsList(planarRegionsList);
      planner.setInitialBodyPose(startPose);
      planner.setGoal(goal);

      planner.setTimeout(10.0);

      assertTrue(planner.planWaypoints().validForExecution());
   }
}
