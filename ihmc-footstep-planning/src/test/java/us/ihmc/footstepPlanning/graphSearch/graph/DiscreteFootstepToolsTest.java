package us.ihmc.footstepPlanning.graphSearch.graph;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class DiscreteFootstepToolsTest
{
   private final Random random = new Random(456789L);
   private final double epsilon = 1e-8;

   @Test
   public void testGetNodeTransform()
   {
      int numTests = 1000;

      for (int i = 0; i < numTests; i++)
      {
         int xLatticeIndex = random.nextInt(1000) - 500;
         int yLatticeIndex = random.nextInt(1000) - 500;
         int yawLatticeIndex = random.nextInt(100) - 50;

         double x = xLatticeIndex * LatticePoint.gridSizeXY;
         double y = yLatticeIndex * LatticePoint.gridSizeXY;
         double yaw = AngleTools.trimAngleMinusPiToPi(yawLatticeIndex * LatticePoint.gridSizeYaw);
         RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

         checkNodeTransform(x, y, yaw, robotSide, 0.0, 0.0, 0.0);
         checkNodeTransform(x, y, yaw, robotSide, 0.4995 * LatticePoint.gridSizeXY, 0.0, 0.0);
         checkNodeTransform(x, y, yaw, robotSide, -0.4995 * LatticePoint.gridSizeXY, 0.0, 0.0);
         checkNodeTransform(x, y, yaw, robotSide, 0.0, 0.4995 * LatticePoint.gridSizeXY, 0.0);
         checkNodeTransform(x, y, yaw, robotSide, 0.0, -0.4995 * LatticePoint.gridSizeXY, 0.0);
         checkNodeTransform(x, y, yaw, robotSide, 0.0, 0.0, 0.4995 * LatticePoint.gridSizeYaw);
         checkNodeTransform(x, y, yaw, robotSide, 0.0, 0.0, -0.4995 * LatticePoint.gridSizeYaw);
      }
   }

   private void checkNodeTransform(double x, double y, double yaw, RobotSide robotSide, double xOffset, double yOffset, double yawOffset)
   {
      DiscreteFootstep node = new DiscreteFootstep(x + xOffset, y + yOffset, yaw + yawOffset, robotSide);

      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      DiscreteFootstepTools.getStepTransform(node, nodeTransform);

      YawPitchRoll rotationYawPitchRoll = new YawPitchRoll();
      rotationYawPitchRoll.set(nodeTransform.getRotation());

      assertEquals(nodeTransform.getTranslationX(), x, epsilon);
      assertEquals(nodeTransform.getTranslationY(), y, epsilon);
      assertEquals(nodeTransform.getTranslationZ(), 0.0, epsilon);
      assertEquals(AngleTools.trimAngleMinusPiToPi(rotationYawPitchRoll.getYaw() - yaw), 0.0, epsilon);
      assertEquals(rotationYawPitchRoll.getPitch(), 0.0, epsilon);
      assertEquals(rotationYawPitchRoll.getRoll(), 0.0, epsilon);
   }

   @Test
   public void testGetSnappedNodeTransform()
   {
      int numTests = 10;

      for (int i = 0; i < numTests; i++)
      {
         double x = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double y = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double yaw = EuclidCoreRandomTools.nextDouble(random, 4.0);
         RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

         DiscreteFootstep node = new DiscreteFootstep(x, y, yaw, robotSide);
         RigidBodyTransform snapTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform snappedNodeTransform = new RigidBodyTransform();
         DiscreteFootstepTools.getSnappedStepTransform(node, snapTransform, snappedNodeTransform);

         RigidBodyTransform expectedSnappedNodeTransform = new RigidBodyTransform();
         DiscreteFootstepTools.getStepTransform(node, expectedSnappedNodeTransform);
         snapTransform.transform(expectedSnappedNodeTransform);

         assertTrue(expectedSnappedNodeTransform.epsilonEquals(snappedNodeTransform, epsilon));
      }
   }

   @Test
   public void testIntersectionChecks1()
   {
      ConvexPolygon2D unitSquare = new ConvexPolygon2D();
      unitSquare.addVertex(-0.5, -0.5);
      unitSquare.addVertex(-0.5, 0.5);
      unitSquare.addVertex(0.5, 0.5);
      unitSquare.addVertex(0.5, -0.5);
      unitSquare.update();

      ConvexPolygon2D testPolygon = new ConvexPolygon2D(unitSquare);
      testPolygon.translate(-0.5, -0.5);
      Assertions.assertTrue(DiscreteFootstepTools.arePolygonsIntersecting(unitSquare, testPolygon));
      testPolygon.translate(0.0, 1.0);
      Assertions.assertTrue(DiscreteFootstepTools.arePolygonsIntersecting(unitSquare, testPolygon));
      testPolygon.translate(1.0, 0.0);
      Assertions.assertTrue(DiscreteFootstepTools.arePolygonsIntersecting(unitSquare, testPolygon));
      testPolygon.translate(0.0, -1.0);
      Assertions.assertTrue(DiscreteFootstepTools.arePolygonsIntersecting(unitSquare, testPolygon));
   }

   @Test
   public void testIntersectionChecks2()
   {
      int numTests = 10000;
      ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

      for (int i = 0; i < numTests; i++)
      {
         ConvexPolygon2D polygon1 = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 2.0, 4);
         ConvexPolygon2D polygon2 = EuclidGeometryRandomTools.nextConvexPolygon2D(random, 2.0, 4);

         double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(polygon1, polygon2);
         boolean intersectionDetected = DiscreteFootstepTools.arePolygonsIntersecting(polygon1, polygon2);

         Assertions.assertEquals(intersectionDetected, intersectionArea > 1e-8);
      }
   }

   @Test
   public void testDistanceChecks1()
   {
      ConvexPolygon2D unitSquare = new ConvexPolygon2D();
      unitSquare.addVertex(-0.5, -0.5);
      unitSquare.addVertex(-0.5, 0.5);
      unitSquare.addVertex(0.5, 0.5);
      unitSquare.addVertex(0.5, -0.5);
      unitSquare.update();

      double distanceToCheck = 0.15;

      ConvexPolygon2D testPolygon = new ConvexPolygon2D(unitSquare);
      testPolygon.translate(-1.0 - distanceToCheck, 0.0);
      Assertions.assertTrue(Math.abs(DiscreteFootstepTools.distanceBetweenPolygons(unitSquare, testPolygon) - distanceToCheck) < epsilon);

      testPolygon = new ConvexPolygon2D(unitSquare);
      testPolygon.translate(1.0 + distanceToCheck, 0.0);
      Assertions.assertTrue(Math.abs(DiscreteFootstepTools.distanceBetweenPolygons(unitSquare, testPolygon) - distanceToCheck) < epsilon);

      testPolygon = new ConvexPolygon2D(unitSquare);
      testPolygon.translate(0.0, 1.0 + distanceToCheck);
      Assertions.assertTrue(Math.abs(DiscreteFootstepTools.distanceBetweenPolygons(unitSquare, testPolygon) - distanceToCheck) < epsilon);

      testPolygon = new ConvexPolygon2D(unitSquare);
      testPolygon.translate(0.0, -1.0 - distanceToCheck);
      Assertions.assertTrue(Math.abs(DiscreteFootstepTools.distanceBetweenPolygons(unitSquare, testPolygon) - distanceToCheck) < epsilon);
   }
}
