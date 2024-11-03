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
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.PathOrientationCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
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

   @Test
   @Disabled
   public void testMaze(TestInfo testInfo)
   {
      WaypointDefinedBodyPathPlanHolder bodyPath = new WaypointDefinedBodyPathPlanHolder();
      List<Pose3D> waypoints = new ArrayList<>();

      ArrayList<PlanarRegion> regions = PointCloudTools.loadPlanarRegionsFromFile("resources/PlanarRegions_NRI_Maze.txt");
      Point3D startPos = new Point3D(9.5, 9, 0);
      Point3D goalPos = new Point3D(0.5, 0.5, 0);
      startPos = PlanarRegionTools.projectPointToPlanes(startPos, new PlanarRegionsList(regions));
      goalPos = PlanarRegionTools.projectPointToPlanes(goalPos, new PlanarRegionsList(regions));

      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(new DefaultVisibilityGraphParameters());
      PathOrientationCalculator orientationCalculator = new PathOrientationCalculator(new DefaultVisibilityGraphParameters());
      List<Point3DReadOnly> pathPoints = navigableRegionsManager.calculateBodyPath(startPos, goalPos);
      List<? extends Pose3DReadOnly> path = orientationCalculator.computePosesFromPath(pathPoints, navigableRegionsManager.getVisibilityMapSolution(),
                                                                                       new Quaternion(), new Quaternion());

      for (Pose3DReadOnly waypoint3d : path)
      {
         waypoints.add(new Pose3D(waypoint3d));
      }
      bodyPath.setPoseWaypoints(path);

      Pose3D startPose = new Pose3D();
      bodyPath.getPointAlongPath(0.0, startPose);
      Pose3D finalPose = new Pose3D();
      bodyPath.getPointAlongPath(1.0, finalPose);

      DefaultFootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      double defaultStepWidth = parameters.getIdealFootstepWidth();

      FramePose3D initialMidFootPose = new FramePose3D();
      initialMidFootPose.setX(startPos.getX());
      initialMidFootPose.setY(startPos.getY());
      initialMidFootPose.getOrientation().setYawPitchRoll(startPose.getYaw(), 0.0, 0.0);
      PoseReferenceFrame midFootFrame = new PoseReferenceFrame("InitialMidFootFrame", initialMidFootPose);

      RobotSide initialStanceFootSide = RobotSide.RIGHT;
      FramePose3D initialStanceFootPose = new FramePose3D(midFootFrame);
      initialStanceFootPose.setY(initialStanceFootSide.negateIfRightSide(defaultStepWidth / 2.0));
      initialStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D goalPose = new FramePose3D();
      goalPose.setX(finalPose.getX());
      goalPose.setY(finalPose.getY());
      goalPose.getOrientation().setYawPitchRoll(finalPose.getYaw(), 0.0, 0.0);

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(regions);
      FootstepPlanningModule planner = new FootstepPlanningModule(getClass().getSimpleName());

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setTimeout(1.0);
      request.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList)));
      request.setStartFootPoses(defaultStepWidth, initialStanceFootPose);
      request.setRequestedInitialStanceSide(initialStanceFootSide);
      request.setGoalFootPoses(defaultStepWidth, goalPose);
      request.getBodyPathWaypoints().addAll(waypoints);

      FootstepPlannerOutput plannerOutput = planner.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());

      if (visualize)
         PlanningTestTools.visualizeAndSleep(planarRegionsList, plannerOutput.getFootstepPlan(), goalPose, bodyPath);
   }
}
