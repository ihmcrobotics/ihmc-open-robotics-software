package us.ihmc.footstepPlanning.graphSearch.aStar;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.tools.PlannerTools;
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

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      FootstepPlanningParameters parameters = new FootstepPlanningParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      planner = AStarFootstepPlanner.createPlanner(parameters, null, footPolygons, expansion, registry);
   }

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
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
            PlannerTools.runPlanner(planner, initialStanceFootPose3d, initialStanceFootSide, goalPose3d, planarRegionsList, !visualize);

      if (visualize)
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose3d);
   }
}
