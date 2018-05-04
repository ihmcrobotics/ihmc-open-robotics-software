package us.ihmc.footstepPlanning.graphSearch.aStar;

import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.polygonSnapping.PlanarRegionsListExamples;
import us.ihmc.footstepPlanning.simplePlanners.FlatGroundPlanningUtils;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AStarPartialFootholdTest
{
   private boolean visualize = true;

   private final YoVariableRegistry registry = new YoVariableRegistry("testRegistry");
   private AStarFootstepPlanner planner;

   private static final double percentageFootholdToTest = 0.5;

   private static final double footWidth = 0.1;
   private static final double footLength = 0.2;
   private static final SideDependentList<ConvexPolygon2D> footPolygons = PlanningTestTools.createFootPolygons(footLength, footWidth);
   private final TestParameters parameters = new TestParameters();

   @Before
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

      ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      planner = AStarFootstepPlanner.createRoughTerrainPlanner(parameters, null, footPolygons, expansion, registry);
      planner.setTimeout(5.0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testPartialFootholds()
   {
      double steppingStoneWidth = footWidth * Math.sqrt(percentageFootholdToTest);
      double steppingStoneLength = footLength * Math.sqrt(percentageFootholdToTest);
      int numberOfSteps = 6;
      PlanarRegionsList planarRegionsList = PlanarRegionsListExamples.generateSteppingStoneField(steppingStoneWidth, steppingStoneLength, parameters.getIdealFootstepWidth(),
                                                                                                 parameters.getIdealFootstepLength(), numberOfSteps);
      planner.setPlanarRegions(planarRegionsList);

      Point2D goalPosition = new Point2D(0.6 + (numberOfSteps + 1) * parameters.getIdealFootstepLength(), 0.0);
      FramePose2D goalPose = new FramePose2D(ReferenceFrame.getWorldFrame(), goalPosition, 0.0);

      FramePose2D initialStanceFootPose = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(), 0.0);
      RobotSide initialStanceFootSide = RobotSide.LEFT;

      FramePose3D initialStanceFootPose3d = FlatGroundPlanningUtils.poseFormPose2d(initialStanceFootPose);
      FramePose3D goalPose3d = FlatGroundPlanningUtils.poseFormPose2d(goalPose);
      FootstepPlan footstepPlan =
            PlanningTestTools.runPlanner(planner, initialStanceFootPose3d, initialStanceFootSide, goalPose3d, planarRegionsList, !visualize);

      if (visualize)
         PlanningTestTools.visualizeAndSleep(planarRegionsList, footstepPlan, goalPose3d);
   }

   private class TestParameters extends DefaultFootstepPlanningParameters
   {
      @Override
      public double getMinimumFootholdPercent()
      {
         return percentageFootholdToTest - 1e-2;
      }
   }
}
