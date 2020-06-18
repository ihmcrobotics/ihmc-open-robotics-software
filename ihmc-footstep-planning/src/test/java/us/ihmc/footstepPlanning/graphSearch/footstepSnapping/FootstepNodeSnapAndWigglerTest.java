package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.Random;

public class FootstepNodeSnapAndWigglerTest
{
   @Test
   public void testQPNotSolvedIfFootSufficientlyInside()
   {
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.setMaximumXYWiggleDistance(0.1);
      footstepPlannerParameters.setMaximumYawWiggle(0.1);
      footstepPlannerParameters.setWiggleInsideDelta(0.05);

      double epsilon = 1e-5;
      double wiggleInsideDelta = footstepPlannerParameters.getWiggleInsideDelta();
      double pX = 0.5 * PlannerTools.footLength;
      double pY = 0.5 * PlannerTools.footWidth;

      ConvexPolygon2D largeEnoughPolygon = new ConvexPolygon2D();
      largeEnoughPolygon.addVertex(pX + (wiggleInsideDelta + epsilon), pY + (wiggleInsideDelta + epsilon));
      largeEnoughPolygon.addVertex(pX + (wiggleInsideDelta + epsilon), -pY - (wiggleInsideDelta + epsilon));
      largeEnoughPolygon.addVertex(-pX - (wiggleInsideDelta + epsilon), pY + (wiggleInsideDelta + epsilon));
      largeEnoughPolygon.addVertex(-pX - (wiggleInsideDelta + epsilon), -pY - (wiggleInsideDelta + epsilon));
      largeEnoughPolygon.update();

      ConvexPolygon2D tooSmallPolygon = new ConvexPolygon2D();
      tooSmallPolygon.addVertex(pX + (wiggleInsideDelta - epsilon), pY + (wiggleInsideDelta - epsilon));
      tooSmallPolygon.addVertex(pX + (wiggleInsideDelta - epsilon), -pY - (wiggleInsideDelta - epsilon));
      tooSmallPolygon.addVertex(-pX - (wiggleInsideDelta - epsilon), pY + (wiggleInsideDelta - epsilon));
      tooSmallPolygon.addVertex(-pX - (wiggleInsideDelta - epsilon), -pY - (wiggleInsideDelta - epsilon));
      tooSmallPolygon.update();

      SideDependentList<ConvexPolygon2D> defaultFootPolygons = PlannerTools.createDefaultFootPolygons();
      FootstepNodeSnapAndWiggleTester snapAndWiggler = new FootstepNodeSnapAndWiggleTester(defaultFootPolygons, footstepPlannerParameters);

      // test region meeting wiggleInsideDelta requirement doesn't call wiggle method
      FootstepNode footstepNode = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
      snapAndWiggler.setPlanarRegions(new PlanarRegionsList(new PlanarRegion(new RigidBodyTransform(), largeEnoughPolygon)));
      snapAndWiggler.snapFootstepNode(footstepNode, null, true);
      Assertions.assertFalse(snapAndWiggler.dirtyBit);

      // test region not meeting wiggleInsideDelta requirement calls wiggle method
      snapAndWiggler.snapFootstepNode(footstepNode);
      snapAndWiggler.setPlanarRegions(new PlanarRegionsList(new PlanarRegion(new RigidBodyTransform(), tooSmallPolygon)));
      snapAndWiggler.snapFootstepNode(footstepNode, null, true);
      Assertions.assertTrue(snapAndWiggler.dirtyBit);
   }

   private class FootstepNodeSnapAndWiggleTester extends FootstepNodeSnapAndWiggler
   {
      boolean dirtyBit = false;

      public FootstepNodeSnapAndWiggleTester(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FootstepPlannerParametersReadOnly parameters)
      {
         super(footPolygonsInSoleFrame, parameters);
      }

      @Override
      protected RigidBodyTransform wiggleIntoConvexHull(ConvexPolygon2D footPolygonInRegionFrame)
      {
         dirtyBit = true;
         return super.wiggleIntoConvexHull(footPolygonInRegionFrame);
      }
   }

   @Test
   public void testMaximumSnapHeightOnFlatRegions()
   {
      double groundHeight = -0.2;
      double maximumSnapHeight = 2.7;

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.translate(0.0, 0.0, groundHeight);
      planarRegionsListGenerator.addRectangle(100.0, 100.0);

      // regions low enough to snap
      planarRegionsListGenerator.identity();
      double lowHeight0 = groundHeight + maximumSnapHeight - 1e-5;
      planarRegionsListGenerator.translate(1.0, -1.0, lowHeight0);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      planarRegionsListGenerator.identity();
      double lowHeight1 = groundHeight + maximumSnapHeight - 0.5;
      planarRegionsListGenerator.translate(1.0, 0.0, lowHeight1);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      planarRegionsListGenerator.identity();
      double lowHeight2 = groundHeight + 0.2;
      planarRegionsListGenerator.translate(1.0, 1.0, lowHeight2);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      // regions too high to snap
      planarRegionsListGenerator.identity();
      double highHeight0 = groundHeight + maximumSnapHeight + 1e-5;
      planarRegionsListGenerator.translate(2.0, -1.0, highHeight0);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      planarRegionsListGenerator.identity();
      double highHeight1 = groundHeight + maximumSnapHeight + 0.5;
      planarRegionsListGenerator.translate(2.0, 0.0, highHeight1);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      planarRegionsListGenerator.identity();
      double highHeight2 = groundHeight + 100.0;
      planarRegionsListGenerator.translate(2.0, 1.0, highHeight2);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);

      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.setMaximumSnapHeight(maximumSnapHeight);
      FootstepNodeSnapAndWiggler snapper  = new FootstepNodeSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), footstepPlannerParameters);
      snapper.setPlanarRegions(planarRegionsList);

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      double epsilon = 1e-8;

      FootstepNode stanceNode = new FootstepNode(0.0, 0.0);
      snapper.snapFootstepNode(stanceNode, null, false);

      // test regions low enough to snap
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(new FootstepNode(1.0, -1.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, lowHeight0));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      snapData = snapper.snapFootstepNode(new FootstepNode(1.0, 0.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, lowHeight1));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      snapData = snapper.snapFootstepNode(new FootstepNode(1.0, 1.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, lowHeight2));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      // test regions high enough to snap
      snapData = snapper.snapFootstepNode(new FootstepNode(2.0, -1.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, groundHeight));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      snapData = snapper.snapFootstepNode(new FootstepNode(2.0, 0.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, groundHeight));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      snapData = snapper.snapFootstepNode(new FootstepNode(2.0, 1.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, groundHeight));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));
   }

   @Test
   public void testMaximumSnapHeightOnSlopedRegion()
   {
      double groundHeight = -0.2;
      double maximumSnapHeight = 2.7;
      double rotatedAngle = Math.toRadians(- 45.0);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.translate(0.0, 0.0, groundHeight);
      planarRegionsListGenerator.addRectangle(100.0, 100.0);
      planarRegionsListGenerator.rotate(rotatedAngle, Axis3D.Y);
      planarRegionsListGenerator.addRectangle(100.0, 100.0);

      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.setMaximumSnapHeight(maximumSnapHeight);
      FootstepNodeSnapAndWiggler snapper  = new FootstepNodeSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), footstepPlannerParameters);
      snapper.setPlanarRegions(planarRegionsList);

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      double epsilon = 1e-8;

      FootstepNode stanceNode = new FootstepNode(0.0, 0.0);
      snapper.snapFootstepNode(stanceNode, null, false);

      FootstepNodeSnapData snapData = snapper.snapFootstepNode(new FootstepNode(- 1.0, 0.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, groundHeight));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      snapData = snapper.snapFootstepNode(new FootstepNode(1.0, 0.0), stanceNode, false);
      Assertions.assertTrue(EuclidCoreTools.epsilonEquals(snapData.getSnapTransform().getRotation().getPitch(), rotatedAngle, epsilon));

      snapData = snapper.snapFootstepNode(new FootstepNode(3.0, 0.0), stanceNode, false);
      Assertions.assertTrue(EuclidCoreTools.epsilonEquals(snapData.getSnapTransform().getRotation().getPitch(), 0.0, epsilon));
   }

   @Test
   public void testOverlapDetection()
   {
      double epsilon = 1e-6;

      ConvexPolygon2D unitSquare = new ConvexPolygon2D();
      unitSquare.addVertex(1.0, 1.0);
      unitSquare.addVertex(1.0, -1.0);
      unitSquare.addVertex(-1.0, 1.0);
      unitSquare.addVertex(-1.0, -1.0);
      unitSquare.update();
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(() -> new ConvexPolygon2D(unitSquare));

      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      parameters.setMinClearanceFromStance(0.0);

      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);

      FootstepNodeSnapData snapData1 = new FootstepNodeSnapData();
      FootstepNodeSnapData snapData2 = new FootstepNodeSnapData();

      FootstepNode node1 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
      FootstepNode node2 = new FootstepNode(0.0, 0.0, 0.0, RobotSide.RIGHT);

      snapData1.getSnapTransform().setIdentity();
      snapData2.getSnapTransform().setIdentity();
      snapData1.getWiggleTransformInWorld().setIdentity();
      snapData2.getWiggleTransformInWorld().setIdentity();

      boolean overlap = snapper.stepsAreTooClose(node1, snapData1, node2, snapData2);
      Assertions.assertTrue(overlap);

      snapData2.getSnapTransform().getTranslation().set(2.0 - epsilon, 0.0, 0.0);
      overlap = snapper.stepsAreTooClose(node1, snapData1, node2, snapData2);
      Assertions.assertTrue(overlap);

      snapData2.getSnapTransform().getTranslation().set(2.0 + epsilon, 0.0, 0.0);
      overlap = snapper.stepsAreTooClose(node1, snapData1, node2, snapData2);
      Assertions.assertFalse(overlap);

      snapData1.getSnapTransform().getRotation().setYawPitchRoll(0.0, 0.0, Math.toRadians(45.0));
      snapData2.getSnapTransform().getTranslation().set(0.0, 1.0 + Math.sqrt(0.5) - epsilon, 0.0);
      overlap = snapper.stepsAreTooClose(node1, snapData1, node2, snapData2);
      Assertions.assertTrue(overlap);

      snapData2.getSnapTransform().getTranslation().set(0.0, 1.0 + Math.sqrt(0.5) + epsilon, 0.0);
      overlap = snapper.stepsAreTooClose(node1, snapData1, node2, snapData2);
      Assertions.assertFalse(overlap);
   }

   @Test
   public void testFullFootholdOverCoplanarRegions1()
   {
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);

      double epsilonDistance = 1e-3;
      parameters.setDistanceEpsilonToBridgeRegions(epsilonDistance);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.translate(-0.5, 0.0, 0.0);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);
      planarRegionsListGenerator.translate(1.0, 0.0, - epsilonDistance / 10.0);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);
      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();

      snapper.setPlanarRegions(planarRegionsList);

      FootstepNode footstepNode = new FootstepNode(0.0, 0.0, 0.0, RobotSide.LEFT);
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(footstepNode);

      double fullArea = footPolygons.get(RobotSide.LEFT).getArea();
      double snappedArea = snapData.getCroppedFoothold().getArea();
      double expectedPercentage = 1.0;
      Assertions.assertTrue(Math.abs(snappedArea / fullArea - expectedPercentage) < 1.0e-6);

      planarRegionsListGenerator.reset();
      planarRegionsListGenerator.translate(-0.5, 0.0, 0.0);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);
      planarRegionsListGenerator.translate(1.0, 0.0, - 1.01 * epsilonDistance);
      planarRegionsListGenerator.addRectangle(1.0, 1.0);
      planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();

      snapper.setPlanarRegions(planarRegionsList);

      snapper.reset();
      snapData = snapper.snapFootstepNode(footstepNode);

      fullArea = footPolygons.get(RobotSide.LEFT).getArea();
      snappedArea = snapData.getCroppedFoothold().getArea();
      expectedPercentage = 0.5;
      Assertions.assertTrue(Math.abs(snappedArea / fullArea - expectedPercentage) < 1.0e-6);
   }

   @Test
   public void testFullFootholdOverCoplanarRegions2()
   {
      Random random = new Random(329023);

      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      DefaultFootstepPlannerParameters parameters = new DefaultFootstepPlannerParameters();
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);

      double epsilonDistance = 1e-3;
      parameters.setDistanceEpsilonToBridgeRegions(epsilonDistance);

      RigidBodyTransform randomXYTranslationAndYaw = new RigidBodyTransform();
      double deltaYaw = LatticeNode.gridSizeYaw;

      randomXYTranslationAndYaw.getTranslation().setX(random.nextInt(100) * LatticeNode.gridSizeXY);
      randomXYTranslationAndYaw.getTranslation().setY(random.nextInt(100) * LatticeNode.gridSizeXY);
      randomXYTranslationAndYaw.getRotation().setYawPitchRoll(deltaYaw, 0.0, 0.0);

      double gap = 0.02;
      double sideLength = 1.0;

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.setTransform(randomXYTranslationAndYaw);
      planarRegionsListGenerator.translate(-0.5 * (sideLength + gap), 0.0, 0.0);
      planarRegionsListGenerator.addRectangle(sideLength, sideLength);
      planarRegionsListGenerator.translate(sideLength + gap, 0.0, -epsilonDistance / 10.0);
      planarRegionsListGenerator.addRectangle(sideLength, sideLength);

      // add some overlapping regions
      planarRegionsListGenerator.translate(-0.5 * (sideLength + gap), 0.5 * (sideLength + gap), 0.0);
      planarRegionsListGenerator.addRectangle(sideLength, sideLength);
      planarRegionsListGenerator.translate(0.0, - (sideLength + gap), 0.0);
      planarRegionsListGenerator.addRectangle(sideLength, sideLength);

      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();
      snapper.setPlanarRegions(planarRegionsList);

      FootstepNode footstepNode = new FootstepNode(randomXYTranslationAndYaw.getTranslationX(),
                                                   randomXYTranslationAndYaw.getTranslationY(),
                                                   randomXYTranslationAndYaw.getRotation().getYaw() + deltaYaw,
                                                   RobotSide.LEFT);
      FootstepNodeSnapData snapData = snapper.snapFootstepNode(footstepNode);

      double fullArea = footPolygons.get(RobotSide.LEFT).getArea();
      double snappedArea = snapData.getCroppedFoothold().getArea();
      double expectedPercentage = 1.0;

      Assertions.assertTrue(Math.abs(snappedArea / fullArea - expectedPercentage) < 1.0e-6);

      gap = 0.0;
      planarRegionsListGenerator.reset();
      planarRegionsListGenerator.setTransform(randomXYTranslationAndYaw);
      planarRegionsListGenerator.translate(-0.5 * (sideLength + gap), 0.0, 0.0);
      planarRegionsListGenerator.addRectangle(sideLength, sideLength);
      planarRegionsListGenerator.translate(sideLength + gap, 0.0, -1.01 * epsilonDistance);
      planarRegionsListGenerator.addRectangle(sideLength, sideLength);

      // add some overlapping regions
      planarRegionsListGenerator.translate(-0.5 * (sideLength + gap), 0.5 * (sideLength + gap), 0.0);
      planarRegionsListGenerator.addRectangle(sideLength, sideLength);
      planarRegionsListGenerator.translate(0.0, - (sideLength + gap), 0.0);
      planarRegionsListGenerator.addRectangle(sideLength, sideLength);

      planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();
      snapper.setPlanarRegions(planarRegionsList);

      footstepNode = new FootstepNode(randomXYTranslationAndYaw.getTranslationX(),
                                      randomXYTranslationAndYaw.getTranslationY(),
                                      randomXYTranslationAndYaw.getRotation().getYaw() + deltaYaw,
                                      RobotSide.LEFT);
      snapData = snapper.snapFootstepNode(footstepNode);

      fullArea = footPolygons.get(RobotSide.LEFT).getArea();
      snappedArea = snapData.getCroppedFoothold().getArea();
      expectedPercentage = 0.5;

      Assertions.assertTrue(Math.abs(snappedArea / fullArea - expectedPercentage) < 1.0e-6);
   }
}
