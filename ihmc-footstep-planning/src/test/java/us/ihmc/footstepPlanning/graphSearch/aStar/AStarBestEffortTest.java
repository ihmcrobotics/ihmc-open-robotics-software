package us.ihmc.footstepPlanning.graphSearch.aStar;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.simplePlanners.FlatGroundPlanningUtils;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import static org.junit.Assert.assertTrue;

public class AStarBestEffortTest
{
   private static final boolean visualize = false;
   private static final boolean assertPlannerReturnedResult = true;

   private final YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
   private AStarFootstepPlanner planner;

   @Before
   public void setup()
   {
      FootstepPlannerParameters parameters = new BestEffortPlannerParameters(3);
      SideDependentList<ConvexPolygon2D> footPolygons = PlanningTestTools.createDefaultFootPolygons();
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      this.planner = AStarFootstepPlanner.createRoughTerrainPlanner(parameters, null, footPolygons, expansion, registry);
      planner.setTimeout(5.0);
   }

   @Test(timeout = 30000)
   public void testBestEffort()
   {
      ConvexPolygon2D groundPlane = new ConvexPolygon2D();
      groundPlane.addVertex(-1.0, -1.0);
      groundPlane.addVertex(-1.0, 1.0);
      groundPlane.addVertex(1.0, -1.0);
      groundPlane.addVertex(1.0, 1.1);
      groundPlane.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(), groundPlane);
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegion);

      Point2D goalPosition = new Point2D(1.5, 0.0);
      FramePose2d goalPose = new FramePose2d(ReferenceFrame.getWorldFrame(), goalPosition, 0.0);

      FramePose2d initialStanceFootPose = new FramePose2d(ReferenceFrame.getWorldFrame());
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(planner, initialStanceFootPose3d, initialStanceFootSide, goalPose3d, planarRegionsList, false);

      if (visualize)
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose3d);

      if (assertPlannerReturnedResult)
      {
         SimpleFootstep footstep = footstepPlan.getFootstep(footstepPlan.getNumberOfSteps() - 1);

         FramePose pose = new FramePose();
         footstep.getSoleFramePose(pose);
         pose.changeFrame(ReferenceFrame.getWorldFrame());
         assertTrue(pose.getPosition().epsilonEquals(new Point3D(1.0, 0.0, 0.0), 0.15));
      }
   }

   private class BestEffortPlannerParameters extends DefaultFootstepPlanningParameters
   {
      private final int minimumStepsForBestEffortPlan;

      BestEffortPlannerParameters(int minimumStepsForBestEffortPlan)
      {
         this.minimumStepsForBestEffortPlan = minimumStepsForBestEffortPlan;
      }

      @Override
      public boolean getReturnBestEffortPlan()
      {
         return true;
      }

      @Override
      public int getMinimumStepsForBestEffortPlan()
      {
         return minimumStepsForBestEffortPlan;
      }
   }
}
