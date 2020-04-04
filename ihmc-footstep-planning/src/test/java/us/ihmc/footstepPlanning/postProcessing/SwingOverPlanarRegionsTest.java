package us.ihmc.footstepPlanning.postProcessing;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPostProcessingPacket;
import io.netty.util.internal.RecyclableArrayList;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsVisualizer;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.postProcessing.parameters.DefaultFootstepPostProcessingParameters;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.LittleWallsWithIncreasingHeightPlanarRegionEnvironment;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

import java.awt.Color;
import java.util.List;
import java.util.stream.Collectors;

import static us.ihmc.robotics.Assert.assertTrue;

public class SwingOverPlanarRegionsTest
{
   private final static boolean visualize = false && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

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

      FootstepPostProcessingPacket result = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
//      checkForCollisions(result, true);
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

      FootstepPostProcessingPacket result = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
//      checkForCollisions(result, true);
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

      FootstepPostProcessingPacket result = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
      checkForCollisions(result, true);
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

      FootstepPostProcessingPacket result = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
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

      FootstepPostProcessingPacket result = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
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

      FootstepPostProcessingPacket processedPacket = runTest(startFoot, endFoot, generator.getPlanarRegionsList());
      checkForCollisions(processedPacket, false);
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

      FootstepPostProcessingPacket processedPacket = runTest(startFoot, endFoot, environment.getPlanarRegionsList());
      checkForCollisions(processedPacket, false);
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

      FootstepPostProcessingPacket processedPacket = runTest(startFoot, endFoot, environment.getPlanarRegionsList());
      checkForCollisions(processedPacket, false);
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

      FootstepPostProcessingPacket processedPacket = runTest(startFoot, endFoot, environment.getPlanarRegionsList());
      checkForCollisions(processedPacket, true);
   }

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

      FootstepPostProcessingPacket processedPacket = runTest(startFoot, endFoot, environment.getPlanarRegionsList());
      checkForCollisions(processedPacket, true);
   }

   private FootstepPostProcessingPacket runTest(FramePose3DReadOnly startFoot, FramePose3DReadOnly endFoot, PlanarRegionsList planarRegionsList)
   {
      WalkingControllerParameters walkingControllerParameters = getWalkingControllerParameters();
      ConvexPolygon2D foot = getFootPolygon();

      Graphics3DObject startGraphics = new Graphics3DObject();
      Graphics3DObject endGraphics = new Graphics3DObject();
      startGraphics.addExtrudedPolygon(foot, 0.02, YoAppearance.Color(Color.blue));
      endGraphics.addExtrudedPolygon(foot, 0.02, YoAppearance.Color(Color.RED));

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

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

      FootstepPostProcessingParametersBasics parameters = getParameters();

      SwingOverRegionsPostProcessingElement swingOverElement = new SwingOverRegionsPostProcessingElement(parameters,
                                                                                                         walkingControllerParameters,
                                                                                                         registry,
                                                                                                         yoGraphicsListRegistry);

      FootstepPostProcessingPacket postProcessingPacket = new FootstepPostProcessingPacket();
      postProcessingPacket.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      FootstepDataMessage firstStep = postProcessingPacket.getFootstepDataList().getFootstepDataList().add();

      firstStep.getLocation().set(endFoot.getPosition());
      firstStep.getOrientation().set(endFoot.getOrientation());
      firstStep.setRobotSide(swingSide.toByte());

      FramePose3D stanceFoot = new FramePose3D(startFoot);
      stanceFoot.getPosition().addY(0.3);

      postProcessingPacket.getLeftFootPositionInWorld().set(stanceFoot.getPosition());
      postProcessingPacket.getLeftFootOrientationInWorld().set(stanceFoot.getOrientation());
      postProcessingPacket.getRightFootPositionInWorld().set(startFoot.getPosition());
      postProcessingPacket.getRightFootOrientationInWorld().set(startFoot.getOrientation());

      PlanarRegionsListDefinedEnvironment environment = new PlanarRegionsListDefinedEnvironment("environment", planarRegionsList, 1e-2, false);

      SwingOverPlanarRegionsTrajectoryExpander expander = swingOverElement.swingOverPlanarRegionsTrajectoryExpander;

      SimulationConstructionSet scs = null;
      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("Dummy"));

         SwingOverPlanarRegionsVisualizer visualizer = new SwingOverPlanarRegionsVisualizer(scs, registry, yoGraphicsListRegistry, foot, expander);
         expander.attachVisualizer(visualizer::update);

         scs.setDT(1.0, 1);
         scs.addYoVariableRegistry(registry);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         scs.setGroundVisible(false);
         scs.addStaticLinkGraphics(environment.getTerrainObject3D().getLinkGraphics());
      }

      FootstepPostProcessingPacket processedPacket = swingOverElement.postProcessFootstepPlan(postProcessingPacket);

      boolean wasAdjusted = expander.wereWaypointsAdjusted();
      if (wasAdjusted)
         assertTrue(processedPacket.getFootstepDataList().getFootstepDataList().get(0).getCustomPositionWaypoints().size() > 0);

      if (wasAdjusted)
      {
         firstWaypoint.set(processedPacket.getFootstepDataList().getFootstepDataList().get(0).getCustomPositionWaypoints().get(0));
         secondWaypoint.set(processedPacket.getFootstepDataList().getFootstepDataList().get(0).getCustomPositionWaypoints().get(1));

         List<FramePoint3D> expandedWaypoints = expander.getExpandedWaypoints();
         for (int i = 0; i < expandedWaypoints.size(); i++)
         {
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expandedWaypoints.get(i),
                                                                 processedPacket.getFootstepDataList()
                                                                                .getFootstepDataList()
                                                                                .get(0)
                                                                                .getCustomPositionWaypoints()
                                                                                .get(i),
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
      return processedPacket;
   }

   private void checkForCollisions(FootstepPostProcessingPacket packet, boolean ignoreGroundSegments)
   {
      SteppingParameters steppingParameters = getWalkingControllerParameters().getSteppingParameters();
      TwoWaypointSwingGenerator twoWaypointSwingGenerator = new TwoWaypointSwingGenerator("",
                                                                                          steppingParameters.getMinSwingHeightFromStanceFoot(),
                                                                                          steppingParameters.getMaxSwingHeightFromStanceFoot(),
                                                                                          steppingParameters.getMinSwingHeightFromStanceFoot(),
                                                                                          new YoVariableRegistry(getClass().getSimpleName()),
                                                                                          null);

      RobotSide swingSide = RobotSide.fromByte(packet.getFootstepDataList().getFootstepDataList().get(0).getRobotSide());

      FramePoint3D stanceFootPosition = new FramePoint3D();
      FramePoint3D swingStartPosition = new FramePoint3D();
      FramePoint3D swingEndPosition = new FramePoint3D();
      FrameOrientation3DBasics swingStartOrientation = new FrameQuaternion();
      FrameOrientation3DBasics swingEndOrientation = new FrameQuaternion();
      FrameVector3D initialVelocity = new FrameVector3D();
      FrameVector3D touchdownVelocity = new FrameVector3D();
      touchdownVelocity.setZ(getWalkingControllerParameters().getSwingTrajectoryParameters().getDesiredTouchdownVelocity());

      if (swingSide == RobotSide.LEFT)
      {
         stanceFootPosition.set(packet.getRightFootPositionInWorld());
         swingStartPosition.set(packet.getLeftFootPositionInWorld());
         swingStartOrientation.set(packet.getLeftFootOrientationInWorld());
      }
      else
      {
         swingStartPosition.set(packet.getRightFootPositionInWorld());
         swingStartOrientation.set(packet.getRightFootOrientationInWorld());
         stanceFootPosition.set(packet.getLeftFootPositionInWorld());
      }
      swingEndPosition.set(packet.getFootstepDataList().getFootstepDataList().get(0).getLocation());
      swingEndOrientation.set(packet.getFootstepDataList().getFootstepDataList().get(0).getOrientation());

      RecyclingArrayList<FramePoint3D> waypoints = new RecyclingArrayList<>(FramePoint3D::new);
      List<Point3D> customWaypoints = packet.getFootstepDataList().getFootstepDataList().get(0).getCustomPositionWaypoints();
      for (int i = 0; i < customWaypoints.size(); i++)
      {
         waypoints.add().set(customWaypoints.get(i));
      }
      twoWaypointSwingGenerator.setStanceFootPosition(stanceFootPosition);
      twoWaypointSwingGenerator.setInitialConditions(swingStartPosition, initialVelocity);
      twoWaypointSwingGenerator.setFinalConditions(swingEndPosition, touchdownVelocity);
      twoWaypointSwingGenerator.setStepTime(1.0);
      twoWaypointSwingGenerator.setTrajectoryType(TrajectoryType.CUSTOM, waypoints);
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

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(packet.getPlanarRegionsList());

      double minDistance =
            Math.max(steppingParameters.getFootBackwardOffset(), steppingParameters.getFootForwardOffset()) + getParameters().getMinimumSwingFootClearance();

      double dt = 1e-3;

      double footLength = getWalkingControllerParameters().getSteppingParameters().getFootLength();
      double toeLength = getWalkingControllerParameters().getSteppingParameters().getFootForwardOffset();
      double heelLength = getWalkingControllerParameters().getSteppingParameters().getFootBackwardOffset();
      double distance = Math.max(Math.max(footLength / 2.0, toeLength), heelLength);


      for (double time = 0.0; time <= 1.0; time += dt)
      {
         twoWaypointSwingGenerator.compute(time);
         FramePoint3D desiredPosition = new FramePoint3D();
         twoWaypointSwingGenerator.getPosition(desiredPosition);

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
               Point3DReadOnly collision = PlanarRegionTools.closestPointOnPlane(desiredPosition, planarRegion);
               FramePoint3D collisionRelativeToEndFoot = new FramePoint3D(ReferenceFrame.getWorldFrame(), collision);
               FramePoint3D collisionRelativeToStartFoot = new FramePoint3D(ReferenceFrame.getWorldFrame(), collision);
               collisionRelativeToEndFoot.changeFrame(endFootPoseFrame);
               collisionRelativeToStartFoot.changeFrame(startFootPoseFrame);

               boolean hittingEndGoal = footPolygon.isPointInside(collisionRelativeToEndFoot.getX(), collisionRelativeToEndFoot.getY()) && Math.abs(collisionRelativeToEndFoot.getZ()) < 0.01;
               boolean hittingStartGoal = footPolygon.isPointInside(collisionRelativeToStartFoot.getX(), collisionRelativeToStartFoot.getY()) && Math.abs(collisionRelativeToStartFoot.getZ()) < 0.01;

               hittingEndGoal |= endFootPolygon.getMinX() < desiredPosition.getX();
               hittingStartGoal |= desiredPosition.getX() < startFootPolygon.getMaxX();

               double distanceToCollision = collision.distance(desiredPosition);
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

   public FootstepPostProcessingParametersBasics getParameters()
   {
      DefaultFootstepPostProcessingParameters parameters = new DefaultFootstepPostProcessingParameters();
      parameters.setDoInitialFastApproximation(true);

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
         public ICPControlGains createICPControlGains()
         {
            return null;
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
         public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters()
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
         public ICPOptimizationParameters getICPOptimizationParameters()
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
         public double defaultOffsetHeightAboveAnkle()
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
         public double getMinSwingHeightFromStanceFoot()
         {
            return 0.10;
         }

         @Override
         public double getDefaultSwingHeightFromStanceFoot()
         {
            return getMinSwingHeightFromStanceFoot();
         }

         @Override
         public double getMaxSwingHeightFromStanceFoot()
         {
            return 0.30;
         }

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
         public double getDesiredStepForward()
         {
            return 0.5; // 0.35;
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
         public double getStepPitch()
         {
            return 0.0;
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
         public double getMinAreaPercentForValidFootstep()
         {
            return 0.5;
         }

         @Override
         public double getDangerAreaPercentForValidFootstep()
         {
            return 0.75;
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
         public boolean doToeTouchdownIfPossible()
         {
            return false;
         }

         @Override
         public double getToeTouchdownAngle()
         {
            return Math.toRadians(20.0);
         }

         @Override
         public boolean doHeelTouchdownIfPossible()
         {
            return false;
         }

         @Override
         public double getHeelTouchdownAngle()
         {
            return Math.toRadians(-5.0);
         }

         @Override
         public double getMinMechanicalLegLength()
         {
            return 0.420;
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
         public double getSwingFootVelocityAdjustmentDamping()
         {
            return 0.8;
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
