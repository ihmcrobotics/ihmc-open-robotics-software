package us.ihmc.footstepPlanning.swing;

import static us.ihmc.robotics.Assert.assertTrue;

import java.awt.Color;
import java.util.List;

import org.apache.commons.lang3.tuple.Pair;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsVisualizer;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.LittleWallsWithIncreasingHeightPlanarRegionEnvironment;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SwingOverPlanarRegionsTest
{
   private static boolean visualize = false;

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   @Test
   public void testAngleStepDown()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.translate(0.0, 0.0, 0.5);
      generator.addRectangle(0.4, 0.4);

      generator.translate(0.5, 0.0, -0.15);
      generator.rotateEuler(new Vector3D(0.0, Math.toRadians(20), 0.0));
      generator.addRectangle(0.4, 0.4);

      double width = 0.25;
      FramePose3D startFoot = new FramePose3D();
      startFoot.getPosition().set(0.0, -width / 2.0, 0.5);

      FramePose3D endFoot = new FramePose3D();
      endFoot.getPosition().set(0.6, -width / 2.0, 0.31);
      endFoot.getOrientation().setYawPitchRoll(0.0, Math.toRadians(20.0), 0.0);

      Pair<FootstepPlannerRequest, FootstepPlan> footstepPlan = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
      //      checkForCollisions(footstepPlan.getKey(), footstepPlan.getRight(), true);
   }

   @Test
   public void testAngleStepSlightPitch()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.translate(0.0, 0.0, 0.5);
      generator.rotateEuler(new Vector3D(0.0, -Math.toRadians(5), 0.0));
      generator.addRectangle(0.4, 0.4);

      generator.identity();
      generator.translate(0.0, 0.0, 0.5);
      generator.translate(0.5, 0.0, -0.15);
      generator.rotateEuler(new Vector3D(0.0, Math.toRadians(20), 0.0));
      generator.addRectangle(0.4, 0.4);

      double width = 0.25;
      FramePose3D startFoot = new FramePose3D();
      startFoot.getPosition().set(0.0, -width / 2.0, 0.5);
      startFoot.getOrientation().setYawPitchRoll(0.0, -Math.toRadians(5), 0.0);

      FramePose3D endFoot = new FramePose3D();
      endFoot.getPosition().set(0.6, -width / 2.0, 0.31);
      endFoot.getOrientation().setYawPitchRoll(0.0, Math.toRadians(20.0), 0.0);

      Pair<FootstepPlannerRequest, FootstepPlan> footstepPlan = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
      //      checkForCollisions(footstepPlan.getKey(), footstepPlan.getRight(), true);
   }

   @Test
   public void testBigStepDown()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.translate(0.0, 0.0, 0.5);
      generator.addRectangle(0.2, 0.4);

      generator.translate(0.2, 0.0, -0.4);
      generator.addRectangle(0.2, 0.4);

      double width = 0.25;
      FramePose3D startFoot = new FramePose3D();
      startFoot.getPosition().set(0.0, -width / 2.0, 0.5);

      FramePose3D endFoot = new FramePose3D();
      endFoot.getPosition().set(0.2, -width / 2.0, 0.1);

      Pair<FootstepPlannerRequest, FootstepPlan> footstepPlan = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
      checkForCollisions(footstepPlan.getKey(), footstepPlan.getRight(), true);
   }

   @Test
   public void testFlatClearance()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      generator.translate(0.4, 0.0, 0.0);
      generator.addRectangle(1.5, 0.4);

      FramePose3D startFoot = new FramePose3D();
      startFoot.getPosition().set(0.0, 0.0, 0.0);

      FramePose3D endFoot = new FramePose3D();
      endFoot.getPosition().set(0.6, 0.0, 0.0);

      Pair<FootstepPlannerRequest, FootstepPlan> footstepPlan = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
   }

   @Test
   public void testFlatClearanceOfCurb()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      ConvexPolygon2D foot = getFootPolygon();

      generator.translate(0.6, 0.0, 0.0);
      generator.addRectangle(1.75, 0.4);

      double cubeDepth = 0.02;
      double cubeHeight = 0.05;
      generator.identity();
      generator.translate(foot.getMaxX() + cubeDepth / 2.0 + 1e-3, 0.0, cubeHeight / 2.0);
      generator.addCubeReferencedAtCenter(cubeDepth, 0.4, cubeHeight);

      FramePose3D startFoot = new FramePose3D();
      startFoot.getPosition().set(0.0, 0.0, 0.0);

      FramePose3D endFoot = new FramePose3D();
      endFoot.getPosition().set(1.0, 0.0, 0.0);

      Pair<FootstepPlannerRequest, FootstepPlan> footstepPlan = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
   }

   @Test
   public void testFlatClearanceOfCurbNoGround()
   {
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();

      ConvexPolygon2D foot = getFootPolygon();

      double cubeDepth = 0.02;
      double cubeHeight = 0.05;
      generator.identity();
      generator.translate(foot.getMaxX() + cubeDepth / 2.0 + 1e-3, 0.0, cubeHeight / 2.0);
      generator.addCubeReferencedAtCenter(cubeDepth, 0.4, cubeHeight);

      FramePose3D startFoot = new FramePose3D();
      startFoot.getPosition().set(0.0, 0.0, 0.0);

      FramePose3D endFoot = new FramePose3D();
      endFoot.getPosition().set(1.0, 0.0, 0.0);

      Pair<FootstepPlannerRequest, FootstepPlan> footstepPlan = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
      checkForCollisions(footstepPlan.getKey(), footstepPlan.getRight(), false);
   }

   @Test
   public void testTrickyStep1()
   {
      LittleWallsWithIncreasingHeightPlanarRegionEnvironment environment = new LittleWallsWithIncreasingHeightPlanarRegionEnvironment();

      ConvexPolygon2D foot = getFootPolygon();

      FramePose3D startFoot = new FramePose3D();
      startFoot.getPosition().set(0.75, 0.05, 0.0);
      startFoot.getOrientation().set(0.0, 0.0, -0.087, 0.996);

      FramePose3D endFoot = new FramePose3D();
      endFoot.getPosition().set(1.4, 0.1, 0.0);
      endFoot.getOrientation().set(0.0, 0.0, 0.174, 0.985);

      Pair<FootstepPlannerRequest, FootstepPlan> footstepPlan = runTest(startFoot, endFoot, environment.getPlanarRegionsList());
      checkForCollisions(footstepPlan.getKey(), footstepPlan.getRight(), false);
   }

   @Test
   public void testTrickyStep2()
   {
      LittleWallsWithIncreasingHeightPlanarRegionEnvironment environment = new LittleWallsWithIncreasingHeightPlanarRegionEnvironment();

      ConvexPolygon2D foot = getFootPolygon();

      FramePose3D startFoot = new FramePose3D();
      startFoot.getPosition().set(1.1, -0.25, 0.0);
      startFoot.getOrientation().set(0.0, 0.0, 0.0, 1.0);

      FramePose3D endFoot = new FramePose3D();
      endFoot.getPosition().set(1.95, -0.1, 0.0);
      endFoot.getOrientation().set(0.0, 0.0, 0.087, 0.996);

      Pair<FootstepPlannerRequest, FootstepPlan> footstepPlan = runTest(startFoot, endFoot, environment.getPlanarRegionsList());
      checkForCollisions(footstepPlan.getKey(), footstepPlan.getRight(), false);
   }

   @Test
   public void testTrickyStep1FullTrajectory()
   {
      LittleWallsWithIncreasingHeightPlanarRegionEnvironment environment = new LittleWallsWithIncreasingHeightPlanarRegionEnvironment(false);

      ConvexPolygon2D foot = getFootPolygon();

      FramePose3D startFoot = new FramePose3D();
      startFoot.getPosition().set(0.75, 0.05, 0.0);
      startFoot.getOrientation().set(0.0, 0.0, -0.087, 0.996);

      FramePose3D endFoot = new FramePose3D();
      endFoot.getPosition().set(1.4, 0.1, 0.0);
      endFoot.getOrientation().set(0.0, 0.0, 0.174, 0.985);

      Pair<FootstepPlannerRequest, FootstepPlan> footstepPlan = runTest(startFoot, endFoot, environment.getPlanarRegionsList());
      checkForCollisions(footstepPlan.getKey(), footstepPlan.getRight(), true);
   }

   @Disabled
   @Test
   public void testTrickyStep2FullTrajectory()
   {
      LittleWallsWithIncreasingHeightPlanarRegionEnvironment environment = new LittleWallsWithIncreasingHeightPlanarRegionEnvironment(false);

      ConvexPolygon2D foot = getFootPolygon();

      FramePose3D startFoot = new FramePose3D();
      startFoot.getPosition().set(1.1, -0.25, 0.0);
      startFoot.getOrientation().set(0.0, 0.0, 0.0, 1.0);

      FramePose3D endFoot = new FramePose3D();
      endFoot.getPosition().set(1.95, -0.1, 0.0);
      endFoot.getOrientation().set(0.0, 0.0, 0.087, 0.996);

      Pair<FootstepPlannerRequest, FootstepPlan> footstepPlan = runTest(startFoot, endFoot, environment.getPlanarRegionsList());
      checkForCollisions(footstepPlan.getKey(), footstepPlan.getRight(), true);
   }

   private Pair<FootstepPlannerRequest, FootstepPlan> runTest(FramePose3DReadOnly startFoot, FramePose3DReadOnly endFoot, PlanarRegionsList planarRegionsList)
   {
      WalkingControllerParameters walkingControllerParameters = getWalkingControllerParameters();
      ConvexPolygon2D foot = getFootPolygon();

      SwingPlannerParametersBasics swingPlannerParameters = getParameters();
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(side -> getFootPolygon());
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName(),
                                                                         new DefaultVisibilityGraphParameters(),
                                                                         new AStarBodyPathPlannerParameters(),
                                                                         new DefaultFootstepPlannerParameters(),
                                                                         swingPlannerParameters,
                                                                         walkingControllerParameters,
                                                                         footPolygons,
                                                                         null);

      Graphics3DObject startGraphics = new Graphics3DObject();
      Graphics3DObject endGraphics = new Graphics3DObject();
      startGraphics.addExtrudedPolygon(foot, 0.02, YoAppearance.Color(Color.blue));
      endGraphics.addExtrudedPolygon(foot, 0.02, YoAppearance.Color(Color.RED));

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      registry.addChild(planningModule.getSwingOverPlanarRegionsTrajectoryExpander().getYoVariableRegistry());
      YoGraphicsListRegistry yoGraphicsListRegistry = planningModule.getSwingOverPlanarRegionsTrajectoryExpander().getGraphicsListRegistry();

      RobotSide swingSide = RobotSide.RIGHT;

      YoFramePoint3D firstWaypoint = new YoFramePoint3D("firstWaypoint", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D secondWaypoint = new YoFramePoint3D("secondWaypoint", ReferenceFrame.getWorldFrame(), registry);

      YoFramePoseUsingYawPitchRoll yoStartFoot = new YoFramePoseUsingYawPitchRoll("start", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoseUsingYawPitchRoll yoEndFoot = new YoFramePoseUsingYawPitchRoll("end", ReferenceFrame.getWorldFrame(), registry);
      yoStartFoot.set(startFoot);
      yoEndFoot.set(endFoot);

      yoGraphicsListRegistry.registerYoGraphic("footsteps", new YoGraphicShape("startFootstep", startGraphics, yoStartFoot, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("footsteps", new YoGraphicShape("endFootstep", endGraphics, yoEndFoot, 1.0));
      yoGraphicsListRegistry.registerYoGraphic("outputWaypoints", new YoGraphicPosition("firstWaypoint", firstWaypoint, 0.02, YoAppearance.White()));
      yoGraphicsListRegistry.registerYoGraphic("outputWaypoints", new YoGraphicPosition("secondWaypoint", secondWaypoint, 0.02, YoAppearance.White()));

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      FootstepPlan footstepPlan = new FootstepPlan();

      PlannedFootstep firstStep = new PlannedFootstep(swingSide);
      firstStep.getFootstepPose().set(endFoot);
      footstepPlan.addFootstep(firstStep);

      FramePose3D stanceFoot = new FramePose3D(startFoot);
      stanceFoot.getPosition().addY(0.3);

      request.getStartFootPoses().get(RobotSide.LEFT).set(stanceFoot);
      request.getStartFootPoses().get(RobotSide.RIGHT).set(startFoot);
      request.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList)));

      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("environment", planarRegionsList, 1e-2, false);

      SwingOverPlanarRegionsTrajectoryExpander expander = planningModule.getSwingOverPlanarRegionsTrajectoryExpander();

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("Dummy"));

         SwingOverPlanarRegionsVisualizer visualizer = new SwingOverPlanarRegionsVisualizer(scs, registry, yoGraphicsListRegistry, foot, expander);
         expander.attachVisualizer(visualizer::update);

         scs.setDT(1.0, 1);
         scs.addYoRegistry(registry);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         scs.setGroundVisible(false);
         scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());
      }

      planningModule.getSwingPlanningModule()
                    .computeSwingWaypoints(request.getHeightMapData(),
                                           footstepPlan,
                                           request.getStartFootPoses(),
                                           SwingPlannerType.TWO_WAYPOINT_POSITION);

      boolean wasAdjusted = expander.wereWaypointsAdjusted();
      if (wasAdjusted)
         assertTrue(footstepPlan.getFootstep(0).getCustomWaypointPositions().size() > 0);

      if (wasAdjusted)
      {
         firstWaypoint.set(footstepPlan.getFootstep(0).getCustomWaypointPositions().get(0));
         secondWaypoint.set(footstepPlan.getFootstep(0).getCustomWaypointPositions().get(1));

         List<FramePoint3D> expandedWaypoints = expander.getExpandedWaypoints();
         for (int i = 0; i < expandedWaypoints.size(); i++)
         {
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expandedWaypoints.get(i),
                                                                 footstepPlan.getFootstep(0).getCustomWaypointPositions().get(i),
                                                                 1e-8);
         }
      }
      else
      {
         firstWaypoint.setToNaN();
         secondWaypoint.setToNaN();
      }

      if (visualize)
      {
         scs.startOnAThread();
         scs.cropBuffer();
         ThreadTools.sleepForever();
      }

      return Pair.of(request, footstepPlan);
   }

   private void checkForCollisions(FootstepPlannerRequest request, PlanarRegionsList planarRegionsList, FootstepPlan footstepPlan, boolean ignoreGroundSegments)
   {
      SwingTrajectoryParameters swingTrajectoryParameters = getWalkingControllerParameters().getSwingTrajectoryParameters();
      TwoWaypointSwingGenerator twoWaypointSwingGenerator = new TwoWaypointSwingGenerator("",
                                                                                          swingTrajectoryParameters.getMinSwingHeight(),
                                                                                          swingTrajectoryParameters.getMaxSwingHeight(),
                                                                                          swingTrajectoryParameters.getMinSwingHeight(),
                                                                                          swingTrajectoryParameters.getCustomWaypointAngleThreshold(),
                                                                                          new YoRegistry(getClass().getSimpleName()),
                                                                                          null);

      RobotSide swingSide = footstepPlan.getFootstep(0).getRobotSide();

      FramePoint3D stanceFootPosition = new FramePoint3D();
      FramePoint3D swingStartPosition = new FramePoint3D();
      FramePoint3D swingEndPosition = new FramePoint3D();
      FrameOrientation3DBasics swingStartOrientation = new FrameQuaternion();
      FrameOrientation3DBasics swingEndOrientation = new FrameQuaternion();
      FrameVector3D initialVelocity = new FrameVector3D();
      FrameVector3D touchdownVelocity = new FrameVector3D();
      touchdownVelocity.setZ(getWalkingControllerParameters().getSwingTrajectoryParameters().getDesiredTouchdownVelocity());

      stanceFootPosition.set(request.getStartFootPoses().get(swingSide.getOppositeSide()).getPosition());
      swingStartPosition.set(request.getStartFootPoses().get(swingSide).getPosition());
      swingStartOrientation.set(request.getStartFootPoses().get(swingSide).getOrientation());
      swingEndPosition.set(footstepPlan.getFootstep(0).getFootstepPose().getPosition());
      swingEndOrientation.set(footstepPlan.getFootstep(0).getFootstepPose().getOrientation());

      List<Point3D> waypoints = footstepPlan.getFootstep(0).getCustomWaypointPositions();
      RecyclingArrayList<FramePoint3D> waypointListCopy = new RecyclingArrayList<>(FramePoint3D.class);
      waypointListCopy.add().set(waypoints.get(0));
      waypointListCopy.add().set(waypoints.get(1));

      twoWaypointSwingGenerator.setStanceFootPosition(stanceFootPosition);
      twoWaypointSwingGenerator.setInitialConditions(swingStartPosition, initialVelocity);
      twoWaypointSwingGenerator.setFinalConditions(swingEndPosition, touchdownVelocity);
      twoWaypointSwingGenerator.setStepTime(1.0);
      twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, waypointListCopy);
      twoWaypointSwingGenerator.initialize();

      PoseReferenceFrame endFootPoseFrame = new PoseReferenceFrame("endFootPoseFrame", ReferenceFrame.getWorldFrame());
      PoseReferenceFrame startFootPoseFrame = new PoseReferenceFrame("startFootPoseFrame", ReferenceFrame.getWorldFrame());
      startFootPoseFrame.setPositionAndUpdate(swingStartPosition);
      startFootPoseFrame.setOrientationAndUpdate(swingStartOrientation);
      endFootPoseFrame.setPositionAndUpdate(swingEndPosition);
      endFootPoseFrame.setOrientationAndUpdate(swingEndOrientation);
      ConvexPolygon2DReadOnly footPolygon = getFootPolygon();
      FrameConvexPolygon2D endFootPolygon = new FrameConvexPolygon2D(endFootPoseFrame, footPolygon);
      FrameConvexPolygon2D startFootPolygon = new FrameConvexPolygon2D(startFootPoseFrame, footPolygon);
      endFootPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());
      startFootPolygon.changeFrameAndProjectToXYPlane(ReferenceFrame.getWorldFrame());

      while (twoWaypointSwingGenerator.doOptimizationUpdate())
         twoWaypointSwingGenerator.compute(0.0);

      double minDistance = getParameters().getMinimumSwingFootClearance();

      double dt = 1e-3;

      double footLength = getWalkingControllerParameters().getSteppingParameters().getFootLength();
      double toeLength = getWalkingControllerParameters().getSteppingParameters().getFootForwardOffset();
      double heelLength = getWalkingControllerParameters().getSteppingParameters().getFootBackwardOffset();
      double distance = Math.max(Math.max(footLength / 2.0, toeLength), heelLength);

      Box3D foot = new Box3D();
      foot.getSize()
          .set(getWalkingControllerParameters().getSteppingParameters().getActualFootLength(),
               getWalkingControllerParameters().getSteppingParameters().getActualFootWidth(),
               0.1);

      for (double time = 0.0; time <= 1.0; time += dt)
      {
         twoWaypointSwingGenerator.compute(time);
         FramePoint3DReadOnly desiredPosition = twoWaypointSwingGenerator.getPosition();

         foot.getPosition().set(desiredPosition);
         foot.getPosition().addX(0.5 * (heelLength - toeLength));

         boolean isInFirstSegment = time < twoWaypointSwingGenerator.getWaypointTime(0);
         boolean isInLastSegment = time > twoWaypointSwingGenerator.getWaypointTime(1);
         boolean highEnoughAboveGround = desiredPosition.getZ() - swingEndPosition.getZ() > distance;
         boolean endingShouldntCheck = isInLastSegment && !highEnoughAboveGround;

         if (!ignoreGroundSegments || (!isInFirstSegment && !endingShouldntCheck))
         {
            double closestDistance = Double.MAX_VALUE;
            Point3DReadOnly closestCollision = null;

            for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
            {
               Point3DReadOnly collision = PlanarRegionTools.closestPointOnPlanarRegion(desiredPosition, planarRegion);
               FramePoint3D collisionRelativeToEndFoot = new FramePoint3D(ReferenceFrame.getWorldFrame(), collision);
               FramePoint3D collisionRelativeToStartFoot = new FramePoint3D(ReferenceFrame.getWorldFrame(), collision);
               collisionRelativeToEndFoot.changeFrame(endFootPoseFrame);
               collisionRelativeToStartFoot.changeFrame(startFootPoseFrame);

               boolean hittingEndGoal = footPolygon.isPointInside(collisionRelativeToEndFoot.getX(), collisionRelativeToEndFoot.getY())
                                        && Math.abs(collisionRelativeToEndFoot.getZ()) < 0.01;
               boolean hittingStartGoal = footPolygon.isPointInside(collisionRelativeToStartFoot.getX(), collisionRelativeToStartFoot.getY())
                                          && Math.abs(collisionRelativeToStartFoot.getZ()) < 0.01;

               hittingEndGoal |= endFootPolygon.getMinX() < desiredPosition.getX();
               hittingStartGoal |= desiredPosition.getX() < startFootPolygon.getMaxX();

               double distanceToCollision = foot.distance(collision);
               if ((!hittingEndGoal && !hittingStartGoal && ignoreGroundSegments) && distanceToCollision < closestDistance)
               {
                  closestDistance = distanceToCollision;
                  closestCollision = collision;
               }
            }
            Assert.assertFalse("have to be " + minDistance + " away, am actually " + closestDistance, (closestDistance + 1.5e-2) < minDistance);
         }
      }
   }

   public SwingPlannerParametersBasics getParameters()
   {
      SwingPlannerParametersBasics parameters = new DefaultSwingPlannerParameters();
      //      parameters.setDoInitialFastApproximation(true);
      parameters.setMinimumSwingFootClearance(0.05);
      return parameters;
   }

   private ConvexPolygon2D getFootPolygon()
   {
      SteppingParameters steppingParameters = getWalkingControllerParameters().getSteppingParameters();

      ConvexPolygon2D foot = new ConvexPolygon2D();
      foot.addVertex(steppingParameters.getFootForwardOffset(), -0.5 * steppingParameters.getToeWidth());
      foot.addVertex(steppingParameters.getFootForwardOffset(), 0.5 * steppingParameters.getToeWidth());
      foot.addVertex(-steppingParameters.getFootBackwardOffset(), -0.5 * steppingParameters.getFootWidth());
      foot.addVertex(-steppingParameters.getFootBackwardOffset(), 0.5 * steppingParameters.getFootWidth());
      foot.update();

      return foot;
   }

   private WalkingControllerParameters getWalkingControllerParameters()
   {
      return new WalkingControllerParameters()
      {
         @Override
         public double getOmega0()
         {
            return 0;
         }

         @Override
         public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
         {
            return false;
         }

         @Override
         public double getMinimumSwingTimeForDisturbanceRecovery()
         {
            return 0;
         }

         @Override
         public double getICPErrorThresholdToSpeedUpSwing()
         {
            return 0;
         }

         @Override
         public boolean allowAutomaticManipulationAbort()
         {
            return false;
         }

         @Override
         public PDGains getCoMHeightControlGains()
         {
            return null;
         }

         @Override
         public PIDSE3Configuration getSwingFootControlGains()
         {
            return null;
         }

         @Override
         public PIDSE3Configuration getHoldPositionFootControlGains()
         {
            return null;
         }

         @Override
         public PIDSE3Configuration getToeOffFootControlGains()
         {
            return null;
         }

         @Override
         public StepAdjustmentParameters getStepAdjustmentParameters()
         {
            return null;
         }

         @Override
         public double getDefaultTransferTime()
         {
            return 0;
         }

         @Override
         public double getDefaultSwingTime()
         {
            return 0;
         }

         @Override
         public FootSwitchFactory getFootSwitchFactory()
         {
            return null;
         }

         @Override
         public String[] getJointsToIgnoreInController()
         {
            return new String[0];
         }

         @Override
         public MomentumOptimizationSettings getMomentumOptimizationSettings()
         {
            return null;
         }

         @Override
         public double getMaxICPErrorBeforeSingleSupportForwardX()
         {
            return 0;
         }

         @Override
         public double getMaxICPErrorBeforeSingleSupportInnerY()
         {
            return 0;
         }

         @Override
         public ToeOffParameters getToeOffParameters()
         {
            return null;
         }

         @Override
         public SwingTrajectoryParameters getSwingTrajectoryParameters()
         {
            return getTestSwingTrajectoryParameters();
         }

         @Override
         public ICPControllerParameters getICPControllerParameters()
         {
            return null;
         }

         @Override
         public double getMaximumLegLengthForSingularityAvoidance()
         {
            return 0;
         }

         @Override
         public double minimumHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double nominalHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double maximumHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public SteppingParameters getSteppingParameters()
         {
            return getTestSteppingParameters();
         }
      };
   }

   private SteppingParameters getTestSteppingParameters()
   {
      return new SteppingParameters()
      {
         @Override
         public double getFootForwardOffset()
         {
            return getFootLength() - getFootBackwardOffset();
         }

         @Override
         public double getFootBackwardOffset()
         {
            return 0.085;
         }

         @Override
         public double getInPlaceWidth()
         {
            return 0.25;
         }

         @Override
         public double getMaxStepLength()
         {
            return 0.6; // 0.5; //0.35;
         }

         @Override
         public double getMinStepWidth()
         {
            return 0.15;
         }

         @Override
         public double getMaxStepWidth()
         {
            return 0.6; // 0.4;
         }

         @Override
         public double getDefaultStepLength()
         {
            return 0.6;
         }

         @Override
         public double getMaxStepUp()
         {
            return 0.25;
         }

         @Override
         public double getMaxStepDown()
         {
            return 0.2;
         }

         @Override
         public double getMaxAngleTurnOutwards()
         {
            //increased atlas turn speed defaults
            // return Math.PI / 4.0;
            return 0.6;
         }

         @Override
         public double getMaxAngleTurnInwards()
         {
            //increased atlas turn speed defaults
            //  return 0;
            return -0.1;
         }

         @Override
         public double getTurningStepWidth()
         {
            return 0.25;
         }

         @Override
         public double getFootWidth()
         {
            return 0.11;
         }

         @Override
         public double getToeWidth()
         {
            return 0.085;
         }

         @Override
         public double getFootLength()
         {
            return 0.22;
         }

         @Override
         public double getActualFootWidth()
         {
            return 0.138;
         }

         @Override
         public double getActualFootLength()
         {
            return 0.26;
         }
      };
   }

   public SwingTrajectoryParameters getTestSwingTrajectoryParameters()
   {
      return new SwingTrajectoryParameters()
      {
         @Override
         public double getMinSwingHeight()
         {
            return 0.10;
         }

         @Override
         public double getDefaultSwingHeight()
         {
            return getMinSwingHeight();
         }

         @Override
         public double getMaxSwingHeight()
         {
            return 0.30;
         }

         @Override
         public double getDesiredTouchdownHeightOffset()
         {
            return 0;
         }

         @Override
         public double getDesiredTouchdownVelocity()
         {
            return -0.3;
         }

         @Override
         public double getDesiredTouchdownAcceleration()
         {
            return -1.0;
         }

         /** {@inheritDoc} */
         @Override
         public boolean addOrientationMidpointForObstacleClearance()
         {
            return false;
         }

         /** {@inheritDoc} */
         @Override
         public boolean useSingularityAvoidanceInSupport()
         {
            return true;
         }
      };
   }
}
