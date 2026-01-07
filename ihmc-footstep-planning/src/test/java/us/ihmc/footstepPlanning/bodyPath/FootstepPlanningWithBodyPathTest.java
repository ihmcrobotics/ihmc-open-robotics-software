package us.ihmc.footstepPlanning.bodyPath;

import org.junit.jupiter.api.*;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class FootstepPlanningWithBodyPathTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final boolean visualize = simulationTestingParameters.getKeepSCSUp();

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testWaypointPathOnFlat(TestInfo testInfo)
   {
      YoRegistry registry = new YoRegistry(testInfo.getTestMethod().get().getName());
      DefaultFootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      double defaultStepWidth = parameters.getIdealFootstepWidth();

      double goalDistance = 5.0;
      FramePose3D initialMidFootPose = new FramePose3D();
      RobotSide initialStanceFootSide = RobotSide.LEFT;
      FramePose3D goalPose = new FramePose3D();
      goalPose.setX(goalDistance);

      WaypointDefinedBodyPathPlanHolder bodyPath = new WaypointDefinedBodyPathPlanHolder();
      List<Point3D> waypoints = new ArrayList<>();
      waypoints.add(new Point3D(0.0, 0.0, 0.0));
      waypoints.add(new Point3D(goalDistance / 8.0, 2.0, 0.0));
      waypoints.add(new Point3D(2.0 * goalDistance / 3.0, -2.0, 0.0));
      waypoints.add(new Point3D(7.0 * goalDistance / 8.0, -2.0, 0.0));
      waypoints.add(new Point3D(goalDistance, 0.0, 0.0));

      bodyPath.setWaypoints(waypoints);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setTimeout(1.0);
      request.setStartFootPoses(defaultStepWidth, initialMidFootPose);
      request.setRequestedInitialStanceSide(initialStanceFootSide);
      request.setGoalFootPoses(defaultStepWidth, goalPose);

      FootstepPlanningModule planner = new FootstepPlanningModule(getClass().getSimpleName());
      FootstepPlannerOutput plannerOutput = planner.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());

      if (visualize)
         PlanningTestTools.visualizeAndSleep(null, plannerOutput.getFootstepPlan(), goalPose, bodyPath);
   }

}
