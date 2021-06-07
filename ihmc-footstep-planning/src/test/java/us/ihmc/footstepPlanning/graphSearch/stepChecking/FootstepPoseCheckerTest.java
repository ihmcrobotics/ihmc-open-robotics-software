package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticePoint;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class FootstepPoseCheckerTest
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   @Test
   public void testStanceFootPitchedTooMuch()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();

      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      FootstepPoseHeuristicChecker checker = new FootstepPoseHeuristicChecker(parameters, snapper, registry);
      parameters.setMaximumStepXWhenFullyPitched(0.3);
      parameters.setMinimumStepZWhenFullyPitched(0.05);

      PlanarRegionsListGenerator planarRegionGenerator = new PlanarRegionsListGenerator();
      planarRegionGenerator.rotate(new Quaternion(0.0, Math.toRadians(-30.0), 0.0));
      planarRegionGenerator.translate(0.0, 0.15, 0.0);
      planarRegionGenerator.addRectangle(0.3, 0.3);
      planarRegionGenerator.identity();
      planarRegionGenerator.translate(0.3, -0.15, -0.15);
      planarRegionGenerator.addRectangle(0.3, 0.3);

      PlanarRegionsList angledGround = planarRegionGenerator.getPlanarRegionsList();

      planarRegionGenerator = new PlanarRegionsListGenerator();
      planarRegionGenerator.translate(0.0, 0.15, 0.0);
      planarRegionGenerator.addRectangle(0.3, 0.3);
      planarRegionGenerator.identity();
      planarRegionGenerator.translate(0.3, -0.15, -0.15);
      planarRegionGenerator.addRectangle(0.3, 0.3);

      PlanarRegionsList flatGround = planarRegionGenerator.getPlanarRegionsList();

      snapper.setPlanarRegions(flatGround);

      DiscreteFootstep stanceNode = new DiscreteFootstep(0.0, 0.15, 0.0, RobotSide.LEFT);
      DiscreteFootstep childNode = new DiscreteFootstep(0.3, -0.15, 0.0, RobotSide.RIGHT);

      BipedalFootstepPlannerNodeRejectionReason rejectionReason = checker.checkStepValidity(childNode, stanceNode, null);
      assertNull(rejectionReason);

      snapper.setPlanarRegions(angledGround);

      rejectionReason = checker.checkStepValidity(childNode, stanceNode, null);
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_LOW_AND_FORWARD_WHEN_PITCHED, rejectionReason);

      // TODO check that this doesn't cause the other rejection reasons to fail if the pitch is flat.
   }

   @Test
   public void testMaxMinYawOnLeftFootAtOrigin()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = yawReduction * maxYaw;
      double minYawAtFullLength = yawReduction * minYaw;

      FootstepPoseHeuristicChecker nodeChecker = new FootstepPoseHeuristicChecker(parameters, snapper, registry);

      double snappedPosition = snapToGrid(parameters.getIdealFootstepWidth());
      double reachAtChild = Math.abs(snappedPosition - parameters.getIdealFootstepWidth());

      double maxValue = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      double minValue = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      DiscreteFootstep parentNode = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.RIGHT);
      DiscreteFootstep childNodeAtMaxYaw = new DiscreteFootstep(0.0, parameters.getIdealFootstepWidth(), snapDownToYaw(maxValue), RobotSide.LEFT);
      DiscreteFootstep childNodeAtMinYaw = new DiscreteFootstep(0.0, parameters.getIdealFootstepWidth(), snapUpToYaw(minValue), RobotSide.LEFT);

      assertNull(nodeChecker.checkStepValidity(childNodeAtMaxYaw, parentNode, null));
      assertNull(nodeChecker.checkStepValidity(childNodeAtMinYaw, parentNode, null));

      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH,
                   nodeChecker.checkStepValidity(new DiscreteFootstep(0.05, parameters.getIdealFootstepWidth(), maxYaw, RobotSide.LEFT), parentNode, null));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH,
                   nodeChecker.checkStepValidity(new DiscreteFootstep(0.05, parameters.getIdealFootstepWidth(), minYaw, RobotSide.LEFT), parentNode, null));
   }

   @Test
   public void testMaxMinYawOnRightFootAtOrigin()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = yawReduction * maxYaw;
      double minYawAtFullLength = yawReduction * minYaw;

      FootstepPoseHeuristicChecker nodeChecker = new FootstepPoseHeuristicChecker(parameters, snapper, registry);

      double snappedPosition = snapToGrid(parameters.getIdealFootstepWidth());
      double reachAtChild = Math.abs(snappedPosition - parameters.getIdealFootstepWidth());

      double maxValue = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      double minValue = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      DiscreteFootstep parentNode = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep childNodeAtMaxYaw = new DiscreteFootstep(0.0, -parameters.getIdealFootstepWidth(), -snapDownToYaw(maxValue), RobotSide.RIGHT);
      DiscreteFootstep childNodeAtMinYaw = new DiscreteFootstep(0.0, -parameters.getIdealFootstepWidth(), -snapUpToYaw(minValue), RobotSide.RIGHT);

      assertNull(nodeChecker.checkStepValidity(childNodeAtMaxYaw, parentNode, null));
      assertNull(nodeChecker.checkStepValidity(childNodeAtMinYaw, parentNode, null));

      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH,
                   nodeChecker.checkStepValidity(new DiscreteFootstep(0.05, -parameters.getIdealFootstepWidth(), -maxYaw, RobotSide.RIGHT), parentNode, null));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH,
                   nodeChecker.checkStepValidity(new DiscreteFootstep(0.05, -parameters.getIdealFootstepWidth(), -minYaw, RobotSide.RIGHT), parentNode, null));
   }

   @Test
   public void testMaxMinYawOnLeftFootAtOriginWithParentYaw()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = yawReduction * maxYaw;
      double minYawAtFullLength = yawReduction * minYaw;

      FootstepPoseHeuristicChecker nodeChecker = new FootstepPoseHeuristicChecker(parameters, snapper, registry);

      double parentYaw = snapToYawGrid(Math.toRadians(75));

      double snappedPosition = snapToGrid(parameters.getIdealFootstepWidth());
      double reachAtChild = Math.abs(snappedPosition - parameters.getIdealFootstepWidth());

      PoseReferenceFrame parentFrame = new PoseReferenceFrame("parentFrame", ReferenceFrame.getWorldFrame());
      parentFrame.setOrientationAndUpdate(new Quaternion(parentYaw, 0.0, 0.0));

      FramePoint3D childPosition = new FramePoint3D(parentFrame, 0.0, parameters.getIdealFootstepWidth(), 0.0);
      childPosition.changeFrame(ReferenceFrame.getWorldFrame());

      double maxValue = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      double minValue = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      DiscreteFootstep parentNode = new DiscreteFootstep(0.0, 0.0, parentYaw, RobotSide.RIGHT);

      DiscreteFootstep childNodeAtMaxYaw = new DiscreteFootstep(childPosition.getX(),
                                                                childPosition.getY(),
                                                                parentYaw + snapDownToYaw(maxValue),
                                                                RobotSide.LEFT);
      DiscreteFootstep childNodeAtMinYaw = new DiscreteFootstep(childPosition.getX(), childPosition.getY(), parentYaw + snapUpToYaw(minValue), RobotSide.LEFT);

      assertNull(nodeChecker.checkStepValidity(childNodeAtMaxYaw, parentNode, null));
      assertNull(nodeChecker.checkStepValidity(childNodeAtMinYaw, parentNode, null));

      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH,
                   nodeChecker.checkStepValidity(new DiscreteFootstep(childPosition.getX(), childPosition.getY(), parentYaw + maxYaw, RobotSide.LEFT),
                                                 parentNode,
                                                 null));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH,
                   nodeChecker.checkStepValidity(new DiscreteFootstep(childPosition.getX(), childPosition.getY(), parentYaw + minYaw, RobotSide.LEFT),
                                                 parentNode,
                                                 null));
   }

   @Test
   public void testMaxMinYawOnRightFootAtOriginWithParentYaw()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = yawReduction * maxYaw;
      double minYawAtFullLength = yawReduction * minYaw;

      FootstepPoseHeuristicChecker nodeChecker = new FootstepPoseHeuristicChecker(parameters, snapper, registry);

      double parentYaw = snapToYawGrid(Math.toRadians(75));

      double snappedPosition = snapToGrid(parameters.getIdealFootstepWidth());
      double reachAtChild = Math.abs(snappedPosition - parameters.getIdealFootstepWidth());

      PoseReferenceFrame parentFrame = new PoseReferenceFrame("parentFrame", ReferenceFrame.getWorldFrame());
      parentFrame.setOrientationAndUpdate(new Quaternion(parentYaw, 0.0, 0.0));

      FramePoint3D childPosition = new FramePoint3D(parentFrame, 0.0, -parameters.getIdealFootstepWidth(), 0.0);
      childPosition.changeFrame(ReferenceFrame.getWorldFrame());

      double maxValue = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      double minValue = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      DiscreteFootstep parentNode = new DiscreteFootstep(0.0, 0.0, parentYaw, RobotSide.LEFT);

      DiscreteFootstep childNodeAtMaxYaw = new DiscreteFootstep(childPosition.getX(),
                                                                childPosition.getY(),
                                                                parentYaw - snapDownToYaw(maxValue),
                                                                RobotSide.RIGHT);
      DiscreteFootstep childNodeAtMinYaw = new DiscreteFootstep(childPosition.getX(), childPosition.getY(), parentYaw - snapUpToYaw(minValue), RobotSide.RIGHT);

      assertNull(nodeChecker.checkStepValidity(childNodeAtMaxYaw, parentNode, null));
      assertNull(nodeChecker.checkStepValidity(childNodeAtMinYaw, parentNode, null));

      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH,
                   nodeChecker.checkStepValidity(new DiscreteFootstep(childPosition.getX(), childPosition.getY(), parentYaw - maxYaw, RobotSide.RIGHT),
                                                 parentNode,
                                                 null));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH,
                   nodeChecker.checkStepValidity(new DiscreteFootstep(childPosition.getX(), childPosition.getY(), parentYaw - minYaw, RobotSide.RIGHT),
                                                 parentNode,
                                                 null));
   }

   @Test
   public void testMaxMinYawOnLeftFootAtOriginSteppingUp()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = yawReduction * maxYaw;
      double minYawAtFullLength = yawReduction * minYaw;

      FootstepPoseHeuristicChecker nodeChecker = new FootstepPoseHeuristicChecker(parameters, snapper, registry);

      double snappedYPosition = snapToGrid(parameters.getIdealFootstepWidth());
      double snappedXPosition = snapDownToGrid(0.8 * parameters.getMaximumStepReach());
      double reachAtChild = EuclidCoreTools.norm(snappedXPosition, snappedYPosition - parameters.getIdealFootstepWidth());

      double maxValue = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      DiscreteFootstep parentNode = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.RIGHT);
      DiscreteFootstep childNodeAtMaxYaw = new DiscreteFootstep(snappedXPosition, parameters.getIdealFootstepWidth(), snapDownToYaw(maxValue), RobotSide.LEFT);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(0.25, 0.15);
      planarRegionsListGenerator.translate(snappedXPosition, snappedYPosition, 0.15);
      planarRegionsListGenerator.addRectangle(0.25, 0.15);

      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();

      snapper.setPlanarRegions(planarRegionsList);

      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH, nodeChecker.checkStepValidity(childNodeAtMaxYaw, parentNode, null));
   }

   private final int queriesPerAxis = 53; // Multiple of 13
   private final double minimumOffsetX = -0.7;
   private final double maximumOffsetX = 0.7;
   private final double minimumOffsetY = -0.4;
   private final double maximumOffsetY = 0.9;
   private final double minimumOffsetYaw = -80.0;
   private final double maximumOffsetYaw = 80.0;

   @Test
   public void testFindNearestReachabilityCheckpoint()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);

      double xSpacing = (maximumOffsetX-minimumOffsetX) / (queriesPerAxis-1);
      double ySpacing = (maximumOffsetY-minimumOffsetY) / (queriesPerAxis-1);
      double yawSpacing = (maximumOffsetYaw-minimumOffsetYaw) / (queriesPerAxis-1);

      Map<FramePose3D, Double> reachabilityMap = updatedPopulateReachabilityMap(queriesPerAxis, minimumOffsetX, maximumOffsetX, minimumOffsetY, maximumOffsetY, minimumOffsetYaw, maximumOffsetYaw);
      FootstepPoseReachabilityChecker reachabilityChecker = new FootstepPoseReachabilityChecker(parameters, snapper, reachabilityMap, registry);
      FramePose3D testFootPose = new FramePose3D();
      FramePose3D nearestCheckpointTrue = new FramePose3D();
      FramePose3D nearestCheckpointCalculated;

      // Test same XY, different yaw
      testFootPose.getPosition().set(2 * xSpacing, ySpacing, 0.0);
      testFootPose.getOrientation().setYawPitchRoll(Math.toRadians(yawSpacing * 2 + 0.001), 0.0, 0.0);
      nearestCheckpointTrue.getPosition().set(2 * xSpacing, ySpacing, 0.0);
      nearestCheckpointTrue.getOrientation().setYawPitchRoll(Math.toRadians(yawSpacing * 2), 0.0, 0.0);
      nearestCheckpointCalculated = reachabilityChecker.updatedFindNearestCheckpoint(testFootPose, reachabilityMap.keySet());
//      assertEquals(nearestCheckpointTrue, nearestCheckpointCalculated);
      assertTrue(nearestCheckpointTrue.geometricallyEquals(nearestCheckpointCalculated, 0.001));

      // Test same X and yaw, different Y
      testFootPose.getPosition().set(-3 * xSpacing, -1 * ySpacing + 0.001, 0.0);
      testFootPose.getOrientation().setYawPitchRoll(Math.toRadians(yawSpacing * 2), 0.0, 0.0);
      nearestCheckpointTrue.getPosition().set(-3 * xSpacing, (-1 * ySpacing), 0.0);
      nearestCheckpointTrue.getOrientation().setYawPitchRoll(Math.toRadians(yawSpacing * 2), 0.0, 0.0);
      nearestCheckpointCalculated = reachabilityChecker.updatedFindNearestCheckpoint(testFootPose, reachabilityMap.keySet());
      assertTrue(nearestCheckpointTrue.geometricallyEquals(nearestCheckpointCalculated, 0.001));

      // Test same Y and yaw, different X
      testFootPose.getPosition().set(12 * xSpacing + 0.001, -5 * ySpacing, 0.0);
      testFootPose.getOrientation().setYawPitchRoll(Math.toRadians(yawSpacing * 7), 0.0, 0.0);
      nearestCheckpointTrue.getPosition().set(12 * xSpacing, -5 * ySpacing, 0.0);
      nearestCheckpointTrue.getOrientation().setYawPitchRoll(Math.toRadians(yawSpacing * 7), 0.0, 0.0);
      nearestCheckpointCalculated = reachabilityChecker.updatedFindNearestCheckpoint(testFootPose, reachabilityMap.keySet());
      assertTrue(nearestCheckpointTrue.geometricallyEquals(nearestCheckpointCalculated, 0.001));

      // Test different XY and yaw
      testFootPose.getPosition().set(8 * xSpacing - 0.001, -3 * ySpacing - 0.001, 0.0);
      testFootPose.getOrientation().setYawPitchRoll(Math.toRadians(yawSpacing * -9 + 0.001), 0.0, 0.0);
      nearestCheckpointTrue.getPosition().set(8 * xSpacing, -3 * ySpacing, 0.0);
      nearestCheckpointTrue.getOrientation().setYawPitchRoll(Math.toRadians(yawSpacing * -9), 0.0, 0.0);
      nearestCheckpointCalculated = reachabilityChecker.updatedFindNearestCheckpoint(testFootPose, reachabilityMap.keySet());
      assertTrue(nearestCheckpointTrue.geometricallyEquals(nearestCheckpointCalculated, 0.001));
   }

   @Test
   public void testCheckpointIsReachable()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      Map<FramePose3D, Double> reachabilityMap = updatedPopulateReachabilityMap(queriesPerAxis, minimumOffsetX, maximumOffsetX, minimumOffsetY, maximumOffsetY, minimumOffsetYaw, maximumOffsetYaw);

      FootstepPoseReachabilityChecker reachabilityChecker = new FootstepPoseReachabilityChecker(parameters, snapper, reachabilityMap, registry);
      FramePose3D testCheckpoint = new FramePose3D();

      // Test unreachable frame in map
      testCheckpoint.getPosition().set(0.431,  0.475,  0.0);
      testCheckpoint.getOrientation().setYawPitchRoll(0.622, 0.0, 0.0);
      assertTrue(reachabilityChecker.checkpointIsReachable(reachabilityMap, testCheckpoint));

      // Test reachable frame in map
      testCheckpoint.getPosition().set(-0.592, -0.300, 0.0);
      testCheckpoint.getOrientation().setYawPitchRoll(-0.392, 0.0, 0.0);
      assertFalse(reachabilityChecker.checkpointIsReachable(reachabilityMap, testCheckpoint));
   }

   private Map<FramePose3D, Double> updatedPopulateReachabilityMap(int queriesPerAxis,
                                                                   double minimumOffsetX,
                                                                   double maximumOffsetX,
                                                                   double minimumOffsetY,
                                                                   double maximumOffsetY,
                                                                   double minimumOffsetYaw,
                                                                   double maximumOffsetYaw)
   {
      Map<FramePose3D, Double> map = new HashMap<>();
      boolean boolSwitch = false;
      minimumOffsetYaw = Math.toRadians(minimumOffsetYaw);
      maximumOffsetYaw = Math.toRadians(maximumOffsetYaw);

      for (int i = 0; i < queriesPerAxis; i++)
      {
         for (int j = 0; j < queriesPerAxis; j++)
         {
            for (int k = 0; k < queriesPerAxis; k++)
            {
               double alphaX = ((double) i) / (queriesPerAxis - 1);
               double alphaY = ((double) j) / (queriesPerAxis - 1);
               double alphaYaw = ((double) k) / (queriesPerAxis - 1);

               double x = EuclidCoreTools.interpolate(minimumOffsetX, maximumOffsetX, alphaX);
               double y = EuclidCoreTools.interpolate(minimumOffsetY, maximumOffsetY, alphaY);
               double yaw = AngleTools.interpolateAngle(minimumOffsetYaw, maximumOffsetYaw, alphaYaw);

               FramePose3D pose = new FramePose3D();
               pose.getPosition().set(x, y, 0.0);
               pose.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);

               // Don't add foot pose where both at origin
               if (pose.getPosition().distanceFromOrigin() != 0)
                  if (boolSwitch) map.put(pose, 20.0);
                  else map.put(pose, 0.0);

               boolSwitch = !boolSwitch;
            }
         }
      }
      return map;
   }

   private static double snapToGrid(double value)
   {
      return LatticePoint.gridSizeXY * Math.round(value / LatticePoint.gridSizeXY);
   }

   private static double snapToYawGrid(double yaw)
   {
      return LatticePoint.gridSizeYaw * Math.floorMod((int) (Math.round((yaw) / LatticePoint.gridSizeYaw)), LatticePoint.yawDivisions);
   }

   private static double snapDownToYaw(double yaw)
   {
      return LatticePoint.gridSizeYaw * Math.floor(yaw / LatticePoint.gridSizeYaw);
   }

   private static double snapDownToGrid(double yaw)
   {
      return LatticePoint.gridSizeXY * Math.floor(yaw / LatticePoint.gridSizeXY);
   }

   private static double snapUpToYaw(double yaw)
   {
      return LatticePoint.gridSizeYaw * Math.ceil(yaw / LatticePoint.gridSizeYaw);
   }
}
