package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.testing.JUnitTools;

public class Cylinder3dTest
{
   double eps = 1e-14;

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCommonShape3dFunctionality()
   {
      Shape3dTestHelper testHelper = new Shape3dTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         RigidBodyTransform transform = RigidBodyTransform.generateRandomTransform(random);
         double height = RandomTools.generateRandomDouble(random, 0.01, 10.0);
         double radius = RandomTools.generateRandomDouble(random, 0.01, 10.0);
         Cylinder3d cylinder3d = new Cylinder3d(transform, height, radius);

         testHelper.runSimpleTests(cylinder3d, random, numberOfPoints);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testCommonShape3dFunctionality_2()
   {
      Shape3dTestHelper testHelper = new Shape3dTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         RigidBodyTransform transform = RigidBodyTransform.generateRandomTransform(random);
         double height = RandomTools.generateRandomDouble(random, 0.01, 10.0);
         double radius = RandomTools.generateRandomDouble(random, 0.01, 10.0);
         Cylinder3d cylinder3d = new Cylinder3d(transform, height, radius);
         Cylinder3d cylinder3d2 = new Cylinder3d(cylinder3d);
         testHelper.runSimpleTests(cylinder3d2, random, numberOfPoints);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGettersAndSetters()
   {
      Shape3dTestHelper testHelper = new Shape3dTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         RigidBodyTransform transform = RigidBodyTransform.generateRandomTransform(random);
         double height = RandomTools.generateRandomDouble(random, 0.01, 10.0);
         double radius = RandomTools.generateRandomDouble(random, 0.01, 10.0);
         Cylinder3d cylinder3d = new Cylinder3d(transform, height, radius);
         
         assertEquals(cylinder3d.getRadius(), radius, 1e-7);
         assertEquals(cylinder3d.getHeight(), height, 1e-7);
         
         cylinder3d.setRadius(5.0);
         cylinder3d.setHeight(10.0);
         
         assertEquals(cylinder3d.getRadius(), 5.0, 1e-7);
         assertEquals(cylinder3d.getHeight(), 10.0, 1e-7);
         
         RigidBodyTransform rbt = new RigidBodyTransform();
         cylinder3d.getTransform(rbt);
         
         Point3d point = new  Point3d();
         rbt.getTranslation(point);
         
         Point3d point1 = new  Point3d();
         transform.getTranslation(point1);
         
         assertEquals(point.getX(), point1.getX(), 1e-7);
         assertEquals(point.getY(), point1.getY(), 1e-7);
         assertEquals(point.getZ(), point1.getZ(), 1e-7);
         
         Quat4d quat1 = new Quat4d();
         rbt.getRotation(quat1);
         
         Quat4d quat2 = new Quat4d();
         transform.getRotation(quat2);
         
         assertEquals(quat1.getX(), quat2.getX(), 1e-7);
         assertEquals(quat1.getY(), quat2.getY(), 1e-7);
         assertEquals(quat1.getZ(), quat2.getZ(), 1e-7);
         assertEquals(quat1.getW(), quat2.getW(), 1e-7);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWithNoTransform()
   {
      double height = 2.0;
      double radius = 0.2;
      Cylinder3d cylinder3d = new Cylinder3d(height, radius);

      Point3d pointToCheck = new Point3d(0.0, 0.0, 0.0001);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(0.0, 0.0, -0.0001);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(0.0, 0.0, height + 0.0001);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(0.0, 0.0, height - 0.0001);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(radius + 0.001, 0.0, height / 2.0);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(radius - 0.001, 0.0, height / 2.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(0.0, radius + 0.001, height / 2.0);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(0.0, radius - 0.001, height / 2.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(radius / 2.0, radius / 2.0, height / 2.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWithTransform()
   {
      // TODO: More tests, rotations.

      double height = 2.0;
      double radius = 0.2;
      RigidBodyTransform transform = new RigidBodyTransform();
      double translateX = 1.1;
      double translateY = 1.3;
      double translateZ = 1.4;
      transform.setTranslation(new Vector3d(translateX, translateY, translateZ));

      Cylinder3d cylinder3d = new Cylinder3d(transform, height, radius);

      Point3d pointToCheck = new Point3d(translateX, translateY, translateZ + 0.0001);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(translateX, translateY, translateZ - 0.0001);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(translateX, translateY, translateZ + height + 0.0001);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(translateX, translateY, translateZ + height - 0.0001);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(translateX + radius + 0.001, translateY, translateZ + height / 2.0);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(translateX + radius - 0.001, translateY, translateZ + height / 2.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(translateX, translateY + radius + 0.001, translateZ + height / 2.0);
      assertFalse(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(translateX, translateY + radius - 0.001, translateZ + height / 2.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));

      pointToCheck = new Point3d(translateX + radius / 2.0, translateY + radius / 2.0, translateZ + height / 2.0);
      assertTrue(cylinder3d.isInsideOrOnSurface(pointToCheck));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOrthogonalProjectionSide()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3d cylinder3d = new Cylinder3d(height, radius);
      Point3d pointToCheckAndPack = new Point3d(1, 1, 1);

      cylinder3d.orthogonalProjection(pointToCheckAndPack);
      double xy = Math.sqrt(radius * radius / 2);
      Point3d expectedProjection = new Point3d(xy, xy, 1);
      assertPointEquals(expectedProjection, pointToCheckAndPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOrthogonalProjectionTop()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3d cylinder3d = new Cylinder3d(height, radius);
      Point3d pointToCheckAndPack = new Point3d(0.5, 0.25, 3);

      Point3d expectedProjection = new Point3d(pointToCheckAndPack.getX(), pointToCheckAndPack.getY(), height);
      cylinder3d.orthogonalProjection(pointToCheckAndPack);
      assertPointEquals(expectedProjection, pointToCheckAndPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOrthogonalProjectionBottom()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3d cylinder3d = new Cylinder3d(height, radius);
      Point3d pointToCheckAndPack = new Point3d(0.5, 0.25, -1);

      Point3d expectedProjection = new Point3d(pointToCheckAndPack.getX(), pointToCheckAndPack.getY(), 0);
      cylinder3d.orthogonalProjection(pointToCheckAndPack);
      assertPointEquals(expectedProjection, pointToCheckAndPack);
   }

   private void assertPointEquals(Point3d expectedPoint, Point3d actualPoint)
   {
      String failMessage = "Expected <(" + expectedPoint.getX() + "," + expectedPoint.getY() + "," + expectedPoint.getZ() + ")>, but was <(" + actualPoint.getX() + ","
                           + actualPoint.getY() + "," + actualPoint.getZ() + ")>";
      assertEquals(failMessage, expectedPoint.getX(), actualPoint.getX(), eps);
      assertEquals(failMessage, expectedPoint.getY(), actualPoint.getY(), eps);
      assertEquals(failMessage, expectedPoint.getZ(), actualPoint.getZ(), eps);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSurfaceNormalAt_OnSide()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3d cylinder3d = new Cylinder3d(height, radius);
      Vector3d normalToPack = new Vector3d();
      Point3d closestPointToPack = new Point3d();

      Point3d pointToCheck = new Point3d(1, 0, 1);
      Vector3d expectedNormal = new Vector3d(1, 0, 0);
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSurfaceNormalAt_inSide()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3d cylinder3d = new Cylinder3d(height, radius);
      Vector3d normalToPack = new Vector3d();
      Point3d closestPointToPack = new Point3d();

      Point3d pointToCheck = new Point3d(0.5, 0, 1);
      Vector3d expectedNormal = new Vector3d(1, 0, 0);
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSurfaceNormalAt_outSide()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3d cylinder3d = new Cylinder3d(height, radius);
      Vector3d normalToPack = new Vector3d();
      Point3d closestPointToPack = new Point3d();

      Point3d pointToCheck = new Point3d(1.5, 0, 1);
      Vector3d expectedNormal = new Vector3d(1, 0, 0);
      assertFalse(cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack));
      assertEquals(expectedNormal, normalToPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSurfaceNormalAt_above()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3d cylinder3d = new Cylinder3d(height, radius);
      Vector3d normalToPack = new Vector3d();
      Point3d closestPointToPack = new Point3d();

      Point3d pointToCheck = new Point3d(0, 0, 3.5);
      Vector3d expectedNormal = new Vector3d(0, 0, 1.0);
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSurfaceNormalAt_below()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3d cylinder3d = new Cylinder3d(height, radius);
      Vector3d normalToPack = new Vector3d();
      Point3d closestPointToPack = new Point3d();

      Point3d pointToCheck = new Point3d(0, 0, -1);
      Vector3d expectedNormal = new Vector3d(0, 0, -1.0);
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSurfaceNormalAt_upAndToSide()
   {
      double height = 2.0;
      double radius = 1.0;
      Cylinder3d cylinder3d = new Cylinder3d(height, radius);
      Vector3d normalToPack = new Vector3d();
      Point3d closestPointToPack = new Point3d();

      Point3d pointToCheck = new Point3d(0.0, 2.0, 3.0);
      Vector3d expectedNormal = new Vector3d(0.0, 1.0, 1.0);
      expectedNormal.normalize();
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSurfaceNormalAt_above_translated()
   {
      double height = 2.0;
      double radius = 1.0;
      RigidBodyTransform transform = new RigidBodyTransform();
      double tx = 1.5, ty = 1, tz = 2;
      Vector3d translation = new Vector3d(tx, ty, tz);
      transform.setTranslationAndIdentityRotation(translation);
      Cylinder3d cylinder3d = new Cylinder3d(transform, height, radius);
      Vector3d normalToPack = new Vector3d();
      Point3d closestPointToPack = new Point3d();

      Point3d pointToCheck = new Point3d(tx, ty, tz + 3.0);
      Vector3d expectedNormal = new Vector3d(0, 0, 1.0);
      cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertEquals(expectedNormal, normalToPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCheckInside()
   {
      double height = 2.0;
      double radius = 1.0;
      RigidBodyTransform transform = new RigidBodyTransform();
      double tx = 1.5, ty = 1, tz = 2;
      Vector3d translation = new Vector3d(tx, ty, tz);
      transform.setTranslationAndIdentityRotation(translation);
      Cylinder3d cylinder3d = new Cylinder3d(transform, height, radius);
      Vector3d normalToPack = new Vector3d();
      Point3d closestPointToPack = new Point3d();

      Point3d pointToCheck = new Point3d(tx, ty, tz + 3.0);
      Vector3d expectedNormal = new Vector3d(0, 0, 1.0);
      boolean isInside = cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      JUnitTools.assertTuple3dEquals(expectedNormal, normalToPack, 1e-7);
      assertFalse(isInside);

      pointToCheck.set(tx, ty, tz + height - 0.011);
      isInside = cylinder3d.checkIfInside(pointToCheck, closestPointToPack, normalToPack);
      assertTrue(isInside);
      JUnitTools.assertTuple3dEquals(expectedNormal, normalToPack, 1e-7);

   }
}
