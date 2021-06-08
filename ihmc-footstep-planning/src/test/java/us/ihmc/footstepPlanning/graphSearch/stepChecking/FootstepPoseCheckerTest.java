package us.ihmc.footstepPlanning.graphSearch.stepChecking;

import org.junit.jupiter.api.Test;
import org.lwjgl.Sys;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityLatticePoint;
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
import us.ihmc.log.LogTools;
import us.ihmc.robotics.Assert;
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

   private final double minimumOffsetX = -0.7;
   private final double maximumOffsetX = 0.7;
   private final double minimumOffsetY = -0.4;
   private final double maximumOffsetY = 0.9;
   private final double minimumOffsetYaw = -Math.toRadians(80.0);
   private final double maximumOffsetYaw = Math.toRadians(80.0);

   private final double spacingXY = 0.05;
   private final int yawDivisions = 10;
   private final double yawSpacing = (maximumOffsetYaw - minimumOffsetYaw) / yawDivisions;

   @Test
   public void testFindNearestLatticePoint()
   {
      StepReachabilityData stepReachabilityData = createTestReachabilityData();

      // Test different X value
      FramePose3D testFootPose = new FramePose3D();
      testFootPose.getPosition().set(2 * spacingXY + 0.001, -5 * spacingXY, 0.0);
      testFootPose.getOrientation().setYawPitchRoll(yawSpacing * 3, 0.0, 0.0);
      StepReachabilityLatticePoint nearestLatticePoint = new StepReachabilityLatticePoint(testFootPose.getX(),
                                                                                          testFootPose.getY(),
                                                                                          testFootPose.getYaw(),
                                                                                          stepReachabilityData.getXySpacing(),
                                                                                          stepReachabilityData.getYawDivisions(),
                                                                                          stepReachabilityData.getGridSizeYaw()
                                                                                          / stepReachabilityData.getYawDivisions());
      assertEquals(2, nearestLatticePoint.getXIndex());
      assertEquals(-5, nearestLatticePoint.getYIndex());
      assertEquals(3, nearestLatticePoint.getYawIndex());

      // Test different Y value
      testFootPose = new FramePose3D();
      testFootPose.getPosition().set(-3 * spacingXY, -2 * spacingXY + 0.004, 0.0);
      testFootPose.getOrientation().setYawPitchRoll(yawSpacing * 1, 0.0, 0.0);
      nearestLatticePoint = new StepReachabilityLatticePoint(testFootPose.getX(),
                                                             testFootPose.getY(),
                                                             testFootPose.getYaw(),
                                                             stepReachabilityData.getXySpacing(),
                                                             stepReachabilityData.getYawDivisions(),
                                                             stepReachabilityData.getGridSizeYaw() / stepReachabilityData.getYawDivisions());
      assertEquals(-3, nearestLatticePoint.getXIndex());
      assertEquals(-2, nearestLatticePoint.getYIndex());
      assertEquals(1, nearestLatticePoint.getYawIndex());

      // Test different yaw value
      testFootPose = new FramePose3D();
      testFootPose.getPosition().set(4 * spacingXY, 2 * spacingXY, 0.0);
      testFootPose.getOrientation().setYawPitchRoll(yawSpacing * 4 + 0.002, 0.0, 0.0);
      nearestLatticePoint = new StepReachabilityLatticePoint(testFootPose.getX(),
                                                             testFootPose.getY(),
                                                             testFootPose.getYaw(),
                                                             stepReachabilityData.getXySpacing(),
                                                             stepReachabilityData.getYawDivisions(),
                                                             stepReachabilityData.getGridSizeYaw() / stepReachabilityData.getYawDivisions());
      assertEquals(4, nearestLatticePoint.getXIndex());
      assertEquals(2, nearestLatticePoint.getYIndex());
      assertEquals(4, nearestLatticePoint.getYawIndex());

      // Test different X,Y, yaw values
      testFootPose = new FramePose3D();
      testFootPose.getPosition().set(-5 * spacingXY - 0.002, 3 * spacingXY + 0.001, 0.0);
      testFootPose.getOrientation().setYawPitchRoll(yawSpacing * 2 + 0.002, 0.0, 0.0);
      nearestLatticePoint = new StepReachabilityLatticePoint(testFootPose.getX(),
                                                             testFootPose.getY(),
                                                             testFootPose.getYaw(),
                                                             stepReachabilityData.getXySpacing(),
                                                             stepReachabilityData.getYawDivisions(),
                                                             stepReachabilityData.getGridSizeYaw() / stepReachabilityData.getYawDivisions());
      assertEquals(-5, nearestLatticePoint.getXIndex());
      assertEquals(3, nearestLatticePoint.getYIndex());
      assertEquals(2, nearestLatticePoint.getYawIndex());
   }

   @Test
   public void testCheckpointIsReachable()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      StepReachabilityData stepReachabilityData = createTestReachabilityData();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      FootstepPoseReachabilityChecker testChecker = new FootstepPoseReachabilityChecker(parameters, snapper, stepReachabilityData, registry);

      StepReachabilityLatticePoint testLatticePoint = new StepReachabilityLatticePoint(-9, 3, 5);
      assertTrue(testChecker.checkpointIsReachable(stepReachabilityData.getLegReachabilityMap(), testLatticePoint));

      testLatticePoint = new StepReachabilityLatticePoint(3, -2, -2);
      assertFalse(testChecker.checkpointIsReachable(stepReachabilityData.getLegReachabilityMap(), testLatticePoint));
   }

   @Test
   public void testLatticePointEquals()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      StepReachabilityData stepReachabilityData = createTestReachabilityData();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);
      FootstepPoseReachabilityChecker testChecker = new FootstepPoseReachabilityChecker(parameters, snapper, stepReachabilityData, registry);

      StepReachabilityLatticePoint testLatticePoint = new StepReachabilityLatticePoint(-9,
                                                                                       3,
                                                                                       5,
                                                                                       stepReachabilityData.getXySpacing(),
                                                                                       stepReachabilityData.getYawDivisions(),
                                                                                       stepReachabilityData.getGridSizeYaw()
                                                                                       / stepReachabilityData.getYawDivisions());
      StepReachabilityLatticePoint identicalLatticePoint = new StepReachabilityLatticePoint(-9,
                                                                                            3,
                                                                                            5,
                                                                                            stepReachabilityData.getXySpacing(),
                                                                                            stepReachabilityData.getYawDivisions(),
                                                                                            stepReachabilityData.getGridSizeYaw()
                                                                                            / stepReachabilityData.getYawDivisions());
      assertEquals(testLatticePoint, identicalLatticePoint);
   }

   private StepReachabilityData createTestReachabilityData()
   {
      StepReachabilityData testReachabilityData = new StepReachabilityData();
      Map<StepReachabilityLatticePoint, Double> reachabilityMap = new HashMap<>();
      boolean boolSwitch = false;

      int minimumXIndex = (int) Math.round(minimumOffsetX / spacingXY);
      int maximumXIndex = (int) Math.round(maximumOffsetX / spacingXY);
      int minimumYIndex = (int) Math.round(minimumOffsetY / spacingXY);
      int maximumYIndex = (int) Math.round(maximumOffsetY / spacingXY);
      int minimumYawIndex = -Math.floorMod((int) (Math.round((minimumOffsetYaw) / yawSpacing)), yawDivisions);
      int maximumYawIndex = Math.floorMod((int) (Math.round((maximumOffsetYaw) / yawSpacing)), yawDivisions);

      for (int xIndex = minimumXIndex; xIndex <= maximumXIndex; xIndex++)
      {
         for (int yIndex = minimumYIndex; yIndex <= maximumYIndex; yIndex++)
         {
            for (int yawIndex = minimumYawIndex; yawIndex <= maximumYawIndex; yawIndex++)
            {
               if (xIndex == 0 && yIndex == 0 && yawIndex == 0)
                  continue;

               StepReachabilityLatticePoint latticePoint = new StepReachabilityLatticePoint(xIndex, yIndex, yawIndex);

               if (boolSwitch)
                  reachabilityMap.put(latticePoint, 20.0);
               else
                  reachabilityMap.put(latticePoint, 0.0);
               boolSwitch = !boolSwitch;
            }
         }
      }
      //      System.out.println(reachabilityMap);
      testReachabilityData.setLegReachabilityMap(reachabilityMap);
      testReachabilityData.setGridData(spacingXY, maximumOffsetYaw - minimumOffsetYaw, yawDivisions);
      return testReachabilityData;
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
