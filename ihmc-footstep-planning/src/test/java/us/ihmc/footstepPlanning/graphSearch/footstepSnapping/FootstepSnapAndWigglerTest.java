package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

public class FootstepSnapAndWigglerTest
{
   @Test
   public void testQPNotSolvedIfFootSufficientlyInside()
   {
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.setMaximumXYWiggleDistance(0.1);
      footstepPlannerParameters.setMaximumYawWiggle(0.1);
      footstepPlannerParameters.setWiggleInsideDeltaTarget(0.05);

      double epsilon = 1e-5;
      double wiggleInsideDelta = footstepPlannerParameters.getWiggleInsideDeltaTarget();
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
      FootstepSnapAndWiggleTester snapAndWiggler = new FootstepSnapAndWiggleTester(defaultFootPolygons, footstepPlannerParameters);

      // test region meeting wiggleInsideDelta requirement doesn't call wiggle method
      DiscreteFootstep footstepNode = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      snapAndWiggler.setPlanarRegions(new PlanarRegionsList(new PlanarRegion(new RigidBodyTransform(), largeEnoughPolygon)));
      snapAndWiggler.snapFootstep(footstepNode, null, true);
      Assertions.assertFalse(snapAndWiggler.dirtyBit);

      // test region not meeting wiggleInsideDelta requirement calls wiggle method
      snapAndWiggler.snapFootstep(footstepNode);
      snapAndWiggler.setPlanarRegions(new PlanarRegionsList(new PlanarRegion(new RigidBodyTransform(), tooSmallPolygon)));
      snapAndWiggler.snapFootstep(footstepNode, null, true);
      Assertions.assertTrue(snapAndWiggler.dirtyBit);
   }

   private class FootstepSnapAndWiggleTester extends FootstepSnapAndWiggler
   {
      boolean dirtyBit = false;

      public FootstepSnapAndWiggleTester(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FootstepPlannerParametersReadOnly parameters)
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
      FootstepSnapAndWiggler snapper  = new FootstepSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), footstepPlannerParameters);
      snapper.setPlanarRegions(planarRegionsList);

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      double epsilon = 1e-8;

      DiscreteFootstep stanceNode = new DiscreteFootstep(0.0, 0.0);
      snapper.snapFootstep(stanceNode, null, false);

      // test regions low enough to snap
      FootstepSnapData snapData = snapper.snapFootstep(new DiscreteFootstep(1.0, -1.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, lowHeight0));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      snapData = snapper.snapFootstep(new DiscreteFootstep(1.0, 0.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, lowHeight1));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      snapData = snapper.snapFootstep(new DiscreteFootstep(1.0, 1.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, lowHeight2));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      // test regions high enough to snap
      snapData = snapper.snapFootstep(new DiscreteFootstep(2.0, -1.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, groundHeight));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      snapData = snapper.snapFootstep(new DiscreteFootstep(2.0, 0.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, groundHeight));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      snapData = snapper.snapFootstep(new DiscreteFootstep(2.0, 1.0), stanceNode, false);
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
      FootstepSnapAndWiggler snapper  = new FootstepSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), footstepPlannerParameters);
      snapper.setPlanarRegions(planarRegionsList);

      RigidBodyTransform expectedTransform = new RigidBodyTransform();
      double epsilon = 1e-8;

      DiscreteFootstep stanceNode = new DiscreteFootstep(0.0, 0.0);
      snapper.snapFootstep(stanceNode, null, false);

      FootstepSnapData snapData = snapper.snapFootstep(new DiscreteFootstep(- 1.0, 0.0), stanceNode, false);
      expectedTransform.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.0, groundHeight));
      Assertions.assertTrue(snapData.getSnapTransform().epsilonEquals(expectedTransform, epsilon));

      snapData = snapper.snapFootstep(new DiscreteFootstep(1.0, 0.0), stanceNode, false);
      Assertions.assertTrue(EuclidCoreTools.epsilonEquals(snapData.getSnapTransform().getRotation().getPitch(), rotatedAngle, epsilon));

      snapData = snapper.snapFootstep(new DiscreteFootstep(3.0, 0.0), stanceNode, false);
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

      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(footPolygons, parameters);

      FootstepSnapData snapData1 = new FootstepSnapData();
      FootstepSnapData snapData2 = new FootstepSnapData();

      DiscreteFootstep node1 = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.LEFT);
      DiscreteFootstep node2 = new DiscreteFootstep(0.0, 0.0, 0.0, RobotSide.RIGHT);

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
   public void testDeltaInsideComputationForConvexRegion()
   {
      ConvexPolygon2D regionPolygon = new ConvexPolygon2D();
      regionPolygon.addVertex(-1.0, -1.0);
      regionPolygon.addVertex(-1.0, 1.0);
      regionPolygon.addVertex(1.0, -1.0);
      regionPolygon.addVertex(1.0, 1.0);
      regionPolygon.update();
      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(), regionPolygon);

      Random random = new Random(392032);
      for (int i = 0; i < 50; i++)
      {
         ConvexPolygon2D stepPolygon = new ConvexPolygon2D(regionPolygon);
         double scale = EuclidCoreRandomTools.nextDouble(random, 0.1, 2.0);
         stepPolygon.scale(scale);
         stepPolygon.update();

         double distance = FootstepSnapAndWiggler.computeAchievedDeltaInside(stepPolygon, planarRegion, false);

         // distance measured from step vertex to closest region edge
         double expectedDistance = (scale > 1.0) ? (- Math.sqrt(2.0) * (scale - 1.0)) : (1.0 - scale);
         double epsilon = 1e-6;

         Assertions.assertTrue(Math.abs(expectedDistance - distance) < epsilon, "FootstepNodeSnapAndWiggler.computeAchievedDeltaInside failing for convex region");
      }
   }

   @Test
   public void testDeltaInsideComputationForConcaveRegion()
   {
      List<Point2D> concaveHullVertices = new ArrayList<>();
      concaveHullVertices.add(new Point2D(-2.0, -2.0));
      concaveHullVertices.add(new Point2D(-2.0, 2.0));
      concaveHullVertices.add(new Point2D(0.0, 0.0));
      concaveHullVertices.add(new Point2D(0.0, 2.0));
      concaveHullVertices.add(new Point2D(2.0, 2.0));
      concaveHullVertices.add(new Point2D(2.0, -2.0));

      ConvexPolygon2D convexHull = new ConvexPolygon2D();
      convexHull.addVertex(-2.0, -2.0);
      convexHull.addVertex(-2.0, 2.0);
      convexHull.addVertex(2.0, -2.0);
      convexHull.addVertex(2.0, 2.0);
      convexHull.update();

      RigidBodyTransform transform = new RigidBodyTransform();
      ConvexPolygon2D stepPolygon = new ConvexPolygon2D(convexHull);
      stepPolygon.scale(0.5);
      stepPolygon.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(), concaveHullVertices, Arrays.asList(convexHull));
      double epsilon = 1e-6;

      double distance = FootstepSnapAndWiggler.computeAchievedDeltaInside(stepPolygon, planarRegion, true);
      double expectedDistance = 0.0;
      Assertions.assertTrue(Math.abs(distance - expectedDistance) < epsilon, "FootstepNodeSnapAndWiggler.computeAchievedDeltaInside failing for concave region");

      double dx = 0.01, dy = 0.0;
      transform.getTranslation().set(dx, dy, 0.0);
      stepPolygon.applyTransform(transform);

      distance = FootstepSnapAndWiggler.computeAchievedDeltaInside(stepPolygon, planarRegion, true);
      expectedDistance = - dx / Math.sqrt(2.0);
      Assertions.assertTrue(Math.abs(distance - expectedDistance) < epsilon, "FootstepNodeSnapAndWiggler.computeAchievedDeltaInside failing for concave region");

      stepPolygon.applyInverseTransform(transform);
      dx = -0.01;
      dy = 0.0;
      transform.getTranslation().set(dx, dy, 0.0);
      stepPolygon.applyTransform(transform);

      distance = FootstepSnapAndWiggler.computeAchievedDeltaInside(stepPolygon, planarRegion, true);
      expectedDistance = - dx / Math.sqrt(2.0);
      Assertions.assertTrue(Math.abs(distance - expectedDistance) < epsilon, "FootstepNodeSnapAndWiggler.computeAchievedDeltaInside failing for concave region");

      stepPolygon.applyInverseTransform(transform);
      dx = 0.0;
      dy = 0.01;
      transform.getTranslation().set(dx, dy, 0.0);
      stepPolygon.applyTransform(transform);

      distance = FootstepSnapAndWiggler.computeAchievedDeltaInside(stepPolygon, planarRegion, true);
      expectedDistance = - dy / Math.sqrt(2.0);
      Assertions.assertTrue(Math.abs(distance - expectedDistance) < epsilon, "FootstepNodeSnapAndWiggler.computeAchievedDeltaInside failing for concave region");

      stepPolygon.applyInverseTransform(transform);
      dx = 0.0;
      dy = -0.01;
      transform.getTranslation().set(dx, dy, 0.0);
      stepPolygon.applyTransform(transform);

      distance = FootstepSnapAndWiggler.computeAchievedDeltaInside(stepPolygon, planarRegion, true);
      expectedDistance = - dy / Math.sqrt(2.0);
      Assertions.assertTrue(Math.abs(distance - expectedDistance) < epsilon, "FootstepNodeSnapAndWiggler.computeAchievedDeltaInside failing for concave region");
   }
}
