package us.ihmc.footstepPlanning.graphSearch.aStar;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.simplePlanners.FlatGroundPlanningUtils;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AStarStartNodeInAVoidTest
{
   private boolean visualize = false;

   private final YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
   private AStarFootstepPlanner planner;

   @Before
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      DefaultFootstepPlanningParameters parameters = new DefaultFootstepPlanningParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = PlanningTestTools.createDefaultFootPolygons();
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      planner = AStarFootstepPlanner.createRoughTerrainPlanner(parameters, null, footPolygons, expansion, registry);
   }

   @Test(timeout = 30000)
   public void testStartNodeInAVoid()
   {
      ConvexPolygon2D groundPlane = new ConvexPolygon2D();
      groundPlane.addVertex(-1.0, -1.0);
      groundPlane.addVertex(-1.0, 1.0);
      groundPlane.addVertex(1.0, -1.0);
      groundPlane.addVertex(1.0, 1.1);
      groundPlane.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(), groundPlane);
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegion);

      Point2D goalPosition = new Point2D(0.5, 0.0);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, 0.0);

      FramePose2D initialStanceFootPose = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(-1.2, 0.0), 0.0);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose3D initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(planner, initialStanceFootPose3d, initialStanceFootSide, goalPose3d, planarRegionsList, !visualize);

      if (visualize)
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose3d);
   }
}
