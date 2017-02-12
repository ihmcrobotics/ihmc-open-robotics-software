package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.testing.JUnitTools;

public class Ellipsoid3dTest
{
   private static final double epsilon = 1e-4;    // This epsilon is meant small changes in coordinates. Use Ellipsoid3d's DEFAULT_EPSILON for error handling.
   private static final int iterations = 100;

	@ContinuousIntegrationTest(estimatedDuration = 0.6)
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
         double xRadius = RandomTools.generateRandomDouble(random, 0.02, 10.0);
         double yRadius = RandomTools.generateRandomDouble(random, 0.02, 10.0);
         double zRadius = RandomTools.generateRandomDouble(random, 0.02, 10.0);
         Ellipsoid3d ellipsoid3d = new Ellipsoid3d(xRadius, yRadius, zRadius, transform);
         testHelper.runSimpleTests(ellipsoid3d, random, numberOfPoints);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleWithNoTransform()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;

      Ellipsoid3d ellipsoid = new Ellipsoid3d(xRadius, yRadius, zRadius);
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xRadius - epsilon, 0.0, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, yRadius - epsilon, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, 0.0, zRadius - epsilon)));

      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(-xRadius + epsilon, 0.0, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, -yRadius + epsilon, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, 0.0, -zRadius + epsilon)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(xRadius + epsilon, 0.0, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, yRadius + epsilon, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, 0.0, zRadius + epsilon)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(-xRadius - epsilon, 0.0, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, -yRadius - epsilon, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, 0.0, -zRadius - epsilon)));
      
      Point3d closestPointToPack = new Point3d();
      Vector3d normalToPack = new Vector3d();
      boolean isInside = ellipsoid.checkIfInside(new Point3d(xRadius - epsilon, 0.0, 0.0), closestPointToPack, normalToPack);
      assertTrue(isInside);
      JUnitTools.assertTuple3dEquals(new Vector3d(1.0, 0.0, 0.0), normalToPack, 1e-7);
      JUnitTools.assertTuple3dEquals(new Point3d(xRadius, 0.0, 0.0), closestPointToPack, 1e-7);
      
      isInside = ellipsoid.checkIfInside(new Point3d(3.0, 7.0, -12.0), closestPointToPack, normalToPack);
      assertFalse(isInside);
      double xScaled = closestPointToPack.getX()/xRadius;
      double yScaled = closestPointToPack.getY()/yRadius;
      double zScaled = closestPointToPack.getZ()/zRadius;
      double sumSquared = xScaled*xScaled + yScaled*yScaled + zScaled*zScaled;
      assertEquals(1.0, sumSquared, 1e-7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleWithTranslationalTransform()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;

      double xTranslation = 0.5;
      double yTranslation = -0.3;
      double zTranslation = 1.1;
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(new Vector3d(xTranslation, yTranslation, zTranslation));

      Ellipsoid3d ellipsoid = new Ellipsoid3d(xRadius, yRadius, zRadius, transform);
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation + xRadius - epsilon, yTranslation, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation, yTranslation + yRadius - epsilon, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation, yTranslation, zTranslation + zRadius - epsilon)));

      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation-xRadius + epsilon, yTranslation, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation, yTranslation -yRadius + epsilon, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation, yTranslation, zTranslation-zRadius + epsilon)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation + xRadius + epsilon, yTranslation, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation, yTranslation + yRadius + epsilon, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation, yTranslation, zTranslation + zRadius + epsilon)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation-xRadius - epsilon, yTranslation, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation, yTranslation-yRadius - epsilon, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation, yTranslation, zTranslation-zRadius - epsilon)));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testExampleUsage()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(Math.PI / 2.0);
      transform.setTranslation(new Vector3d(0.0, 5.0, 0.0));

      Ellipsoid3d ellipsoid = new Ellipsoid3d(xRadius, yRadius, zRadius, transform);

      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, 8.0, 0.0), 0.001));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, 7.9, 0.0), 0.001));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, 8.2, 0.0), 0.001));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(0.2, 8.2, 0.2), 0.001));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSet()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;

      Ellipsoid3d ellipsoid = new Ellipsoid3d(xRadius, yRadius, zRadius);

      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, 0.0, 0.0)));

      double[] zeroes = new double[] {0.0, 0.0, 0.0};

      double[] radii = new double[] {xRadius, yRadius, zRadius};

      assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, zeroes, zeroes, ellipsoid);
      assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, zeroes, new double[] {-epsilon, -epsilon, -epsilon}, ellipsoid);
      assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(true, radii, zeroes, new double[] {epsilon, epsilon, epsilon}, ellipsoid);
   }

   private void assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(boolean invertResult, double[] radii, double[] translations, double[] offsets,
           Ellipsoid3d ellipsoid)
   {
      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3d(translations[0] + radii[0] + offsets[0], translations[1], translations[2])));
      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3d(translations[0] - (radii[0] + offsets[0]), translations[1], translations[2])));

      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3d(translations[0], translations[1] + radii[1] + offsets[1], translations[2])));
      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3d(translations[0], translations[1] - (radii[1] + offsets[1]), translations[2])));

      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3d(translations[0], translations[1], translations[2] + radii[2] + offsets[2])));
      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3d(translations[0], translations[1], translations[2] - (radii[2] + offsets[2]))));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTranslation()
   {
      for (int i = 0; i < iterations; i++)
      {
         Random random = new Random(1776L);

         double xRadius = random.nextDouble();
         double yRadius = random.nextDouble();
         double zRadius = random.nextDouble();

         double[] radii = new double[] {xRadius, yRadius, zRadius};

         double xTranslation = random.nextDouble();
         double yTranslation = random.nextDouble();
         double zTranslation = random.nextDouble();

         double[] translations = new double[] {xTranslation, yTranslation, zTranslation};

         double[] zeroes = new double[] {0.0, 0.0, 0.0};

         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(new Vector3d(xTranslation, yTranslation, zTranslation));

         Ellipsoid3d ellipsoid = new Ellipsoid3d(xRadius, yRadius, zRadius);
         ellipsoid.setTransform(transform);

         assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xTranslation, yTranslation, zTranslation)));

         assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, translations, zeroes, ellipsoid);
         assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, translations, new double[] {-epsilon, -epsilon, -epsilon}, ellipsoid);
         assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(true, radii, translations, new double[] {epsilon, epsilon, epsilon}, ellipsoid);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimpleRotations()
   {
      for (int n = 0; n < iterations; n++)
      {
         Random random = new Random(1776L);

         double xRadius = random.nextDouble();
         double yRadius = random.nextDouble();
         double zRadius = random.nextDouble();

         double[] angles = new double[] {Math.PI / 3.0, Math.PI / 4.0, Math.PI / 6.0};

         for (double angle : angles)
         {
            for (int i = 0; i < 3; i++)
            {
               RigidBodyTransform transform = new RigidBodyTransform();

               if (i == 0)
                  transform.setRotationRollAndZeroTranslation(angle);
               if (i == 1)
                  transform.setRotationPitchAndZeroTranslation(angle);
               if (i == 2)
                  transform.setRotationYawAndZeroTranslation(angle);

               Ellipsoid3d ellipsoid = new Ellipsoid3d(xRadius, yRadius, zRadius);
               ellipsoid.setTransform(transform);

               assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, 0.0, 0.0)));

               if (i == 0)
               {
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xRadius, 0.0, 0.0)));

                  double yCoord = yRadius * Math.cos(angle);
                  double zCoord = yRadius * Math.sin(angle);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, yCoord, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, -yCoord, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, yCoord + epsilon, zCoord + epsilon)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, -(yCoord + epsilon), -(zCoord + epsilon))));

                  yCoord = zRadius * Math.cos(angle + Math.PI / 2.0);
                  zCoord = zRadius * Math.sin(angle + Math.PI / 2.0);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, yCoord, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, -yCoord, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, yCoord - epsilon, zCoord + epsilon)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, -(yCoord - epsilon), -(zCoord + epsilon))));
               }

               if (i == 1)
               {
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, yRadius, 0.0)));

                  double xCoord = xRadius * Math.cos(angle);
                  double zCoord = -xRadius * Math.sin(angle);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xCoord, 0.0, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(-xCoord, 0.0, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(xCoord + epsilon, 0.0, zCoord - epsilon)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(-(xCoord + epsilon), 0.0, -(zCoord - epsilon))));

                  xCoord = zRadius * Math.cos(angle + Math.PI / 2.0);
                  zCoord = -zRadius * Math.sin(angle + Math.PI / 2.0);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xCoord, 0.0, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(-xCoord, 0.0, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(xCoord - epsilon, 0.0, zCoord - epsilon)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(-(xCoord - epsilon), 0.0, -(zCoord - epsilon))));
               }

               if (i == 2)
               {
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(0.0, 0.0, zRadius)));

                  double xCoord = xRadius * Math.cos(angle);
                  double yCoord = xRadius * Math.sin(angle);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xCoord, yCoord, 0.0)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(-xCoord, -yCoord, 0.0)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(xCoord + epsilon, yCoord + epsilon, 0.0)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(-(xCoord + epsilon), -(yCoord + epsilon), 0.0)));

                  xCoord = yRadius * Math.cos(angle + Math.PI / 2.0);
                  yCoord = yRadius * Math.sin(angle + Math.PI / 2.0);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(xCoord, yCoord, 0.0)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3d(-xCoord, -yCoord, 0.0)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(xCoord - epsilon, yCoord + epsilon, 0.0)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3d(-(xCoord - epsilon), -(yCoord + epsilon), 0.0)));
               }
            }
         }
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testMethodsForRandomEllipsoids()
   {
      Random random = new Random(1865L);
      Ellipsoid3d ellipsoid = new Ellipsoid3d(1.0, 2.0, 3.0);
      double xRad, yRad, zRad;
      RigidBodyTransform transform = randomTransform(random);

      for (int n = 0; n < iterations; n++)
      {
         xRad = random.nextDouble();
         yRad = random.nextDouble();
         zRad = random.nextDouble();

         ellipsoid.setXRadius(xRad);
         ellipsoid.setYRadius(yRad);
         ellipsoid.setZRadius(zRad);
         ellipsoid.setTransform(transform);

         Ellipsoid3d ellipsoidCopy = new Ellipsoid3d(ellipsoid);

         assertEquals(ellipsoid.getXRadius(), ellipsoidCopy.getXRadius(), 1e-10);
         assertEquals(ellipsoid.getYRadius(), ellipsoidCopy.getYRadius(), 1e-10);
         assertEquals(ellipsoid.getZRadius(), ellipsoidCopy.getZRadius(), 1e-10);

         Point3d center = new Point3d();
         ellipsoid.getCenter(center);

         Point3d centerCopy = new Point3d();
         ellipsoidCopy.getCenter(centerCopy);

         JUnitTools.assertTuple3dEquals(center, centerCopy, 1e-10);
      }
   }

   private RigidBodyTransform randomTransform(Random random)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      RigidBodyTransform tempTransform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(2 * Math.PI * random.nextDouble());
      tempTransform.setRotationPitchAndZeroTranslation(2 * Math.PI * random.nextDouble());
      transform.multiply(tempTransform);
      tempTransform.setRotationYawAndZeroTranslation(2 * Math.PI * random.nextDouble());
      transform.multiply(tempTransform);

      double[] matrix = new double[16];
      transform.get(matrix);
      matrix[3] = random.nextDouble();
      matrix[7] = random.nextDouble();
      matrix[11] = random.nextDouble();

      return transform;
   }
}
