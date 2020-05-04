package us.ihmc.footstepPlanning.graphSearch.nodeChecking;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class GoodFootstepPositionCheckerTest
{
   private final FootstepPlannerEdgeData edgeData = new FootstepPlannerEdgeData();

   @Test
   public void testStanceFootPitchedTooMuch()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();

      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      GoodFootstepPositionChecker checker = new GoodFootstepPositionChecker(parameters, snapper, edgeData);
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

      FootstepNode stanceNode = new FootstepNode(0.0, 0.15, 0.0, RobotSide.LEFT);
      FootstepNode childNode = new FootstepNode(0.3, -0.15, 0.0, RobotSide.RIGHT);

      boolean isValid = checker.isNodeValid(childNode, stanceNode);
      BipedalFootstepPlannerNodeRejectionReason rejectionReason = checker.getRejectionReason();
      assertTrue(isValid);
      assertEquals(null, rejectionReason);

      snapper.setPlanarRegions(angledGround);

      isValid = checker.isNodeValid(childNode, stanceNode);
      assertFalse(isValid);
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_TOO_LOW_AND_FORWARD_WHEN_PITCHED, checker.getRejectionReason());

      // TODO check that this doesn't cause the other rejection reasons to fail if the pitch is flat.
   }

   @Test
   public void testMaxMinYawOnLeftFootAtOrigin()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = yawReduction * maxYaw;
      double minYawAtFullLength = yawReduction * minYaw;

      GoodFootstepPositionChecker nodeChecker = new GoodFootstepPositionChecker(parameters, snapper, edgeData);

      double snappedPosition = snapToGrid(parameters.getIdealFootstepWidth());
      double reachAtChild = Math.abs(snappedPosition - parameters.getIdealFootstepWidth());

      double maxValue = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      double minValue = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      FootstepNode parentNode = new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT);
      FootstepNode childNodeAtMaxYaw = new FootstepNode(0.0, parameters.getIdealFootstepWidth(), snapDownToYaw(maxValue), RobotSide.LEFT);
      FootstepNode childNodeAtMinYaw = new FootstepNode(0.0, parameters.getIdealFootstepWidth(), snapUpToYaw(minValue), RobotSide.LEFT);

      assertTrue(nodeChecker.isNodeValid(childNodeAtMaxYaw, parentNode));
      assertTrue(nodeChecker.isNodeValid(childNodeAtMinYaw, parentNode));

      assertFalse(nodeChecker.isNodeValid(new FootstepNode(0.05, parameters.getIdealFootstepWidth(), maxYaw, RobotSide.LEFT), parentNode));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH, nodeChecker.getRejectionReason());
      assertFalse(nodeChecker.isNodeValid(new FootstepNode(0.05, parameters.getIdealFootstepWidth(), minYaw, RobotSide.LEFT), parentNode));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH, nodeChecker.getRejectionReason());
   }

   @Test
   public void testMaxMinYawOnRightFootAtOrigin()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = yawReduction * maxYaw;
      double minYawAtFullLength = yawReduction * minYaw;

      GoodFootstepPositionChecker nodeChecker = new GoodFootstepPositionChecker(parameters, snapper, edgeData);

      double snappedPosition = snapToGrid(parameters.getIdealFootstepWidth());
      double reachAtChild = Math.abs(snappedPosition - parameters.getIdealFootstepWidth());

      double maxValue = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      double minValue = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      FootstepNode parentNode = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
      FootstepNode childNodeAtMaxYaw = new FootstepNode(0.0, -parameters.getIdealFootstepWidth(), -snapDownToYaw(maxValue), RobotSide.RIGHT);
      FootstepNode childNodeAtMinYaw = new FootstepNode(0.0, -parameters.getIdealFootstepWidth(), -snapUpToYaw(minValue), RobotSide.RIGHT);

      assertTrue(nodeChecker.isNodeValid(childNodeAtMaxYaw, parentNode));
      assertTrue(nodeChecker.isNodeValid(childNodeAtMinYaw, parentNode));

      assertFalse(nodeChecker.isNodeValid(new FootstepNode(0.05, -parameters.getIdealFootstepWidth(), -maxYaw, RobotSide.RIGHT), parentNode));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH, nodeChecker.getRejectionReason());
      assertFalse(nodeChecker.isNodeValid(new FootstepNode(0.05, -parameters.getIdealFootstepWidth(), -minYaw, RobotSide.RIGHT), parentNode));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH, nodeChecker.getRejectionReason());
   }

   @Test
   public void testMaxMinYawOnLeftFootAtOriginWithParentYaw()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = yawReduction * maxYaw;
      double minYawAtFullLength = yawReduction * minYaw;

      GoodFootstepPositionChecker nodeChecker = new GoodFootstepPositionChecker(parameters, snapper, edgeData);

      double parentYaw = snapToYawGrid(Math.toRadians(75));

      double snappedPosition = snapToGrid(parameters.getIdealFootstepWidth());
      double reachAtChild = Math.abs(snappedPosition - parameters.getIdealFootstepWidth());

      PoseReferenceFrame parentFrame = new PoseReferenceFrame("parentFrame", ReferenceFrame.getWorldFrame());
      parentFrame.setOrientationAndUpdate(new Quaternion(parentYaw, 0.0, 0.0));

      FramePoint3D childPosition = new FramePoint3D(parentFrame, 0.0, parameters.getIdealFootstepWidth(), 0.0);
      childPosition.changeFrame(ReferenceFrame.getWorldFrame());

      double maxValue = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      double minValue = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      FootstepNode parentNode = new FootstepNode(0.0, 0.0, parentYaw, RobotSide.RIGHT);

      FootstepNode childNodeAtMaxYaw = new FootstepNode(childPosition.getX(), childPosition.getY(), parentYaw + snapDownToYaw(maxValue), RobotSide.LEFT);
      FootstepNode childNodeAtMinYaw = new FootstepNode(childPosition.getX(), childPosition.getY(), parentYaw + snapUpToYaw(minValue), RobotSide.LEFT);

      assertTrue(nodeChecker.isNodeValid(childNodeAtMaxYaw, parentNode));
      assertTrue(nodeChecker.isNodeValid(childNodeAtMinYaw, parentNode));

      assertFalse(nodeChecker.isNodeValid(new FootstepNode(childPosition.getX(), childPosition.getY(), parentYaw + maxYaw, RobotSide.LEFT), parentNode));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH, nodeChecker.getRejectionReason());
      assertFalse(nodeChecker.isNodeValid(new FootstepNode(childPosition.getX(), childPosition.getY(), parentYaw + minYaw, RobotSide.LEFT), parentNode));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH, nodeChecker.getRejectionReason());
   }

   @Test
   public void testMaxMinYawOnRightFootAtOriginWithParentYaw()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = yawReduction * maxYaw;
      double minYawAtFullLength = yawReduction * minYaw;

      GoodFootstepPositionChecker nodeChecker = new GoodFootstepPositionChecker(parameters, snapper, edgeData);

      double parentYaw = snapToYawGrid(Math.toRadians(75));

      double snappedPosition = snapToGrid(parameters.getIdealFootstepWidth());
      double reachAtChild = Math.abs(snappedPosition - parameters.getIdealFootstepWidth());

      PoseReferenceFrame parentFrame = new PoseReferenceFrame("parentFrame", ReferenceFrame.getWorldFrame());
      parentFrame.setOrientationAndUpdate(new Quaternion(parentYaw, 0.0, 0.0));

      FramePoint3D childPosition = new FramePoint3D(parentFrame, 0.0, -parameters.getIdealFootstepWidth(), 0.0);
      childPosition.changeFrame(ReferenceFrame.getWorldFrame());

      double maxValue = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      double minValue = InterpolationTools.linearInterpolate(minYaw, minYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      FootstepNode parentNode = new FootstepNode(0.0, 0.0, parentYaw, RobotSide.LEFT);

      FootstepNode childNodeAtMaxYaw = new FootstepNode(childPosition.getX(), childPosition.getY(), parentYaw - snapDownToYaw(maxValue), RobotSide.RIGHT);
      FootstepNode childNodeAtMinYaw = new FootstepNode(childPosition.getX(), childPosition.getY(), parentYaw - snapUpToYaw(minValue), RobotSide.RIGHT);

      assertTrue(nodeChecker.isNodeValid(childNodeAtMaxYaw, parentNode));
      assertTrue(nodeChecker.isNodeValid(childNodeAtMinYaw, parentNode));

      assertFalse(nodeChecker.isNodeValid(new FootstepNode(childPosition.getX(), childPosition.getY(), parentYaw - maxYaw, RobotSide.RIGHT), parentNode));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH, nodeChecker.getRejectionReason());
      assertFalse(nodeChecker.isNodeValid(new FootstepNode(childPosition.getX(), childPosition.getY(), parentYaw - minYaw, RobotSide.RIGHT), parentNode));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH, nodeChecker.getRejectionReason());
   }

   @Test
   public void testMaxMinYawOnLeftFootAtOriginSteppingUp()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      double maxYaw = 1.2;
      double minYaw = -0.5;
      double yawReduction = 0.5;
      parameters.setMaximumStepYaw(maxYaw);
      parameters.setMinimumStepYaw(minYaw);
      parameters.setStepYawReductionFactorAtMaxReach(yawReduction);

      double maxYawAtFullLength = yawReduction * maxYaw;
      double minYawAtFullLength = yawReduction * minYaw;

      GoodFootstepPositionChecker nodeChecker = new GoodFootstepPositionChecker(parameters, snapper, edgeData);

      double snappedYPosition = snapToGrid(parameters.getIdealFootstepWidth());
      double snappedXPosition = snapDownToGrid(0.8 * parameters.getMaximumStepReach());
      double reachAtChild = EuclidCoreTools.norm(snappedXPosition, snappedYPosition - parameters.getIdealFootstepWidth());

      double maxValue = InterpolationTools.linearInterpolate(maxYaw, maxYawAtFullLength, reachAtChild / parameters.getMaximumStepReach());
      FootstepNode parentNode = new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT);
      FootstepNode childNodeAtMaxYaw = new FootstepNode(snappedXPosition, parameters.getIdealFootstepWidth(), snapDownToYaw(maxValue), RobotSide.LEFT);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(0.25, 0.15);
      planarRegionsListGenerator.translate(snappedXPosition, snappedYPosition, 0.15);
      planarRegionsListGenerator.addRectangle(0.25, 0.15);

      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();

      snapper.setPlanarRegions(planarRegionsList);

      assertFalse(nodeChecker.isNodeValid(childNodeAtMaxYaw, parentNode));
      assertEquals(BipedalFootstepPlannerNodeRejectionReason.STEP_YAWS_TOO_MUCH, nodeChecker.getRejectionReason());
   }


   private static double snapToGrid(double value)
   {
      return LatticeNode.gridSizeXY * Math.round(value / LatticeNode.gridSizeXY);
   }

   private static double snapToYawGrid(double yaw)
   {
      return LatticeNode.gridSizeYaw * Math.floorMod((int) (Math.round((yaw) / LatticeNode.gridSizeYaw)), LatticeNode.yawDivisions);
   }

   private static double snapDownToYaw(double yaw)
   {
      return LatticeNode.gridSizeYaw * Math.floor(yaw / LatticeNode.gridSizeYaw);
   }

   private static double snapDownToGrid(double yaw)
   {
      return LatticeNode.gridSizeXY * Math.floor(yaw / LatticeNode.gridSizeXY);
   }

   private static double snapUpToYaw(double yaw)
   {
      return LatticeNode.gridSizeYaw * Math.ceil(yaw / LatticeNode.gridSizeYaw);
   }
}
