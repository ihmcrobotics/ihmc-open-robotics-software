package us.ihmc.robotics.geometry.shapes;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Torus3dTest
{
   private final double MIN_THICKNESS = 0.005;
   private final double epsilon = 0.0001;
   private final int iterations = 1000;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testExapleUsage()
   {
      double radius = 1.0;
      double thickness = 0.1;
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(Math.PI/2.0);
      transform.setTranslation(new Vector3D(2.0, 0.0, 3.0));
      
      Torus3d torus3d = new Torus3d(transform, radius, thickness);
      Point3D pointToCheck = new Point3D(2.0, 0.0, 4.0);
      
      assertTrue(torus3d.isInsideOrOnSurface(pointToCheck));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimplePointOnOrInside()
   {
      double radius = 1.0;
      double thickness = 0.1;
      
      Torus3d torus3d = new Torus3d(radius, thickness);
      testPointsInsideWhenOffsetBy(torus3d, 0.0, 0.0, 0.0);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTranslatedPointsOnOrInside()
   {
      double radius = 1.0;
      double thickness = 0.1;
      
      testPointsInsideWhenTranslated(radius, thickness);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testPointsInsideRandomSizesAndThicknesses()
   {
      Random random = new Random(1888L);

      for (int i = 0; i < iterations; i++)
      {
         double[] radiusAndThickness = getRandomRadiusAndThickness(random);
         
//         System.out.println("Torus3dTest:testRandomSizes: radius=" + radius + ", thickness=" + thickness);

         double radius = radiusAndThickness[0];
         double thickness = radiusAndThickness[1];
         
         Torus3d torus3d = new Torus3d(radius, thickness);
         testPointsInsideWhenOffsetBy(torus3d, 0.0, 0.0, 0.0);
         
         testTranslatedPointsOnOrInside();
         testPointsInsideWhenTranslated(radius, thickness);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOrthogonalProjection()
   {
      double radius = 1.0;
      double thickness = 0.1;
    
      Torus3d torus3d = new Torus3d(radius, thickness);
      
      Point3D testPoint = new Point3D(1.0, 0.0, 0.0);
      Point3D projectedPoint = new Point3D(testPoint);
      torus3d.orthogonalProjection(projectedPoint);
      
      EuclidCoreTestTools.assertTuple3DEquals(testPoint, projectedPoint, 1e-7);
      
      testPoint = new Point3D(-1.09, 0.0, 0.0);
      projectedPoint = new Point3D(testPoint);
      torus3d.orthogonalProjection(projectedPoint);
      
      EuclidCoreTestTools.assertTuple3DEquals(testPoint, projectedPoint, 1e-7);
     
      testPoint = new Point3D(radius + 0.2, 0.0, 0.0);
      projectedPoint = new Point3D(testPoint);
      torus3d.orthogonalProjection(projectedPoint);
      Point3D expectedProjectedPoint = new Point3D(radius+thickness, 0.0, 0.0);
      
      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-7);
      
      double amountPastCenter = 1.7;
      testPoint = new Point3D(radius + amountPastCenter, 0.0, amountPastCenter);
      projectedPoint = new Point3D(testPoint);
      torus3d.orthogonalProjection(projectedPoint);
      expectedProjectedPoint = new Point3D(radius + thickness * Math.sqrt(2.0)/2.0, 0.0, thickness * Math.sqrt(2.0)/2.0);
      
      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-7);
      
      // Middle of torus should project to anywhere on the inside ring.
      testPoint = new Point3D(0.0, 0.0, 0.0);
      projectedPoint = new Point3D(testPoint);
      torus3d.orthogonalProjection(projectedPoint);
      
      assertEquals(radius-thickness, testPoint.distance(projectedPoint), 1e-7);
      assertEquals(0.0, projectedPoint.getZ(), 1e-7); 
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testClosestPointAndNormalAt()
   {
      // also tests orthogonalProjection() and surfaceNormalAt()
      double radius = 1.0;
      double thickness = 0.1;
      Torus3d torus3d = new Torus3d(radius, thickness);
      Point3D closestPointToPack = new Point3D();
      Vector3D normalToPack = new Vector3D();
 
      // any value on the axis of the origin of the torus is indeterminate, but will be treated as being at (R, 0, 0).
      //X
      // on the outside edge
      Point3D pointInWorldToCheck = new Point3D(radius+thickness, 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius+thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(1.0, 0.0, 0.0), 10e-7));
      
      // beyond the outside edge
      pointInWorldToCheck = new Point3D(radius+thickness+epsilon, 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius+thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(1.0, 0.0, 0.0), 10e-7));

      // on the outside edge
      pointInWorldToCheck = new Point3D(-(radius+thickness), 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-(radius+thickness), 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(-1.0, 0.0, 0.0), 10e-7));

      // beyond the outside edge
      pointInWorldToCheck = new Point3D(-(radius+thickness+epsilon), 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-(radius+thickness), 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(-1.0, 0.0, 0.0), 10e-7));

      // on the inner edge
      pointInWorldToCheck = new Point3D(radius-thickness, 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius-thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(-1.0, 0.0, 0.0), 10e-7));

      // beyond the inner edge
      pointInWorldToCheck = new Point3D(radius-(thickness+epsilon), 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius-thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(-1.0, 0.0, 0.0), 10e-7));

      // on the inner edge
      pointInWorldToCheck = new Point3D(-radius+thickness, 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-radius+thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(1.0, 0.0, 0.0), 10e-7));

      // beyond the inner edge
      pointInWorldToCheck = new Point3D(-radius+(thickness+epsilon), 0.0, 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-radius+thickness, 0.0, 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(1.0, 0.0, 0.0), 10e-7));
      
      //Y
      // on the outside edge
      pointInWorldToCheck = new Point3D(0.0, (radius+thickness), 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(0.0, (radius+thickness), 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 1.0, 0.0), 10e-7));
   
      // beyond the outside edge
      pointInWorldToCheck = new Point3D(0.0, (radius+thickness+epsilon), 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(0.0, (radius+thickness), 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 1.0, 0.0), 10e-7));

      // on the outside edge
      pointInWorldToCheck = new Point3D(0.0, -(radius+thickness), 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(0.0, -(radius+thickness), 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, -1.0, 0.0), 10e-7));

      // beyond the outside edge
      pointInWorldToCheck = new Point3D(0.0, -(radius+thickness+epsilon), 0.0);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(0.0, -(radius+thickness), 0.0), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, -1.0, 0.0), 10e-7));

      // Z at X=radius
      pointInWorldToCheck = new Point3D(radius, 0.0, thickness);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius, 0.0, thickness), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 0.0, 1.0), 10e-7));
      
      pointInWorldToCheck = new Point3D(radius, 0.0, thickness+epsilon);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(radius, 0.0, thickness), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 0.0, 1.0), 10e-7));

      pointInWorldToCheck = new Point3D(-radius, 0.0, -thickness);
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-radius, 0.0, -thickness), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 0.0, -1.0), 10e-7));

      pointInWorldToCheck = new Point3D(-radius, 0.0, -(thickness+epsilon));
      torus3d.checkIfInside(pointInWorldToCheck, closestPointToPack, normalToPack);
      assertTrue(closestPointToPack.epsilonEquals(new Point3D(-radius, 0.0, -thickness), 10e-7));
      assertTrue(normalToPack.epsilonEquals(new Vector3D(0.0, 0.0, -1.0), 10e-7));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test90DegRotation()
   {
      Random random = new Random(1972L);

      for (int n = 0; n < iterations; n++)
      {
         double[] radiusAndThickness = getRandomRadiusAndThickness(random);
         double radius = radiusAndThickness[0];
         double thickness = radiusAndThickness[1];
         
         Torus3d torus3d = new Torus3d(radius, thickness);
         
         // center point should always be false
         assertFalse(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));
         
         RigidBodyTransform transform = new RigidBodyTransform();
         RotationMatrix rotation = new RotationMatrix();
         
         // test rotation about x-axis of pi/2
         rotation.setToRollMatrix(Math.PI / 2);
         transform.setRotation(rotation);
         torus3d.setTransform(transform);
         
         assertFalse(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(radius, 0.0, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(radius, 0.0, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, radius)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, -radius)));
         
         // test rotation about y-axis of pi/2
         rotation.setToPitchMatrix(Math.PI / 2);
         transform.setRotation(rotation);
         torus3d.setTransform(transform);
         
         assertFalse(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, radius, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, radius, 0.0)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, radius)));
         assertTrue(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, -radius)));
         
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testSimpleRotations()
   {
      Random random = new Random(1984L);
      
      double radius = 1.0;
      double thickness = 0.1;
      
      for (int n = 0; n < iterations; n++)
      {
         while (true)
         {
         double[] radiusAndThickness = getRandomRadiusAndThickness(random);
         radius = radiusAndThickness[0];
         thickness = radiusAndThickness[1];
         
         thickness = thickness * Math.abs(Math.cos(Math.PI / 3)) - 0.01; // limit thickness of torus so that edges do not accidentally intersect non-rotated axes
         if (thickness >= MIN_THICKNESS && radius > MIN_THICKNESS + epsilon)
            break;
         }
         
         Torus3d torus3d = new Torus3d(radius, thickness);
         
         // center point should always be false
         assertFalse(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));
         
         RigidBodyTransform transform = new RigidBodyTransform();
         RotationMatrix rotation = new RotationMatrix();
         
         // loop to test rotations of pi/3, pi/4, and pi/6 about x and y-axes respectively
         double[] angles = new double[] { Math.PI / 3, Math.PI / 4, Math.PI / 6, 
                                          2 * Math.PI / 3, 3 * Math.PI / 4, 5 * Math.PI / 6 };
         
         for (double angle : angles)
         {
            //System.out.println("Radius = " + radius + " thickness = " + thickness);
            
            torus3d.set(new Torus3d(radius, thickness));

            assertFalse(torus3d.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));

            for (int i = 0; i < 2; i++)
            {
               if (i == 0)
               {
                  rotation.setToRollMatrix(angle);
//                  System.out.println("Rotating " + angle + " rads about X");
               }
               if (i == 1)
               {
                  rotation.setToPitchMatrix(angle);
//                  System.out.println("Rotating " + angle + " rads about Y");
               }

               transform.setRotation(rotation);
               torus3d.setTransform(transform);

               for (int j = -1; j < 2; j++)
               {
                  double[] xyzToTest = new double[] { 0.0, 0.0, 0.0 };
                  xyzToTest[i] = radius + (j * thickness); // testing radius - thickness, radius, and radius + thickness

//                  for (double point : xyzToTest)
//                     System.out.print(point + " ");
//                  System.out.print("\n");
                  
                  // only points along rotating axis should not move
                  assertTrue(torus3d.isInsideOrOnSurface(new Point3D(xyzToTest[0], xyzToTest[1], xyzToTest[2])));
                  assertTrue(torus3d.isInsideOrOnSurface(new Point3D(-xyzToTest[0], -xyzToTest[1], -xyzToTest[2])));
                  assertFalse(torus3d.isInsideOrOnSurface(new Point3D(xyzToTest[1], xyzToTest[0], xyzToTest[2])));
                  assertFalse(torus3d.isInsideOrOnSurface(new Point3D(-xyzToTest[1], -xyzToTest[0], -xyzToTest[2])));
                  assertFalse(torus3d.isInsideOrOnSurface(new Point3D(xyzToTest[1], xyzToTest[2], xyzToTest[0])));
                  assertFalse(torus3d.isInsideOrOnSurface(new Point3D(-xyzToTest[1], -xyzToTest[2], -xyzToTest[0])));

                  xyzToTest[i] = 0.0;

                  if (i == 0)
                     xyzToTest[1] = (radius + j * thickness) * Math.cos(angle);
                  if (i == 1)
                     xyzToTest[0] = (radius + j * thickness) * Math.cos(angle);

                  xyzToTest[2] = (radius + j * thickness) * Math.sin(angle) * (i == 1 ? -1 : 1);
                  
//                  for (double point : xyzToTest)
//                     System.out.print(point + " ");
//                  System.out.print("\n");

                  // points along non-rotated axis should be displaced in 2 dimensions
                  assertTrue(torus3d.isInsideOrOnSurface(new Point3D(xyzToTest[0], xyzToTest[1], xyzToTest[2])));
                  assertTrue(torus3d.isInsideOrOnSurface(new Point3D(-xyzToTest[0], -xyzToTest[1], -xyzToTest[2])));
               }
            }
         }
      }
   }
   
   private double[] getRandomRadiusAndThickness(Random random)
   {
     double radius = random.nextDouble() * 10.0;
      while (radius <= MIN_THICKNESS + epsilon)
      {
         radius = random.nextDouble() * 10.0;
      }
      
      double thickness = random.nextDouble() * radius;
      while (thickness < MIN_THICKNESS || radius - thickness <= epsilon)
      {
         thickness = random.nextDouble() * radius;
      }
      
      return new double[]{radius, thickness};
   }
   
   public void testPointsInsideWhenTranslated(double radius, double thickness)
   {
      Torus3d torus3d = new Torus3d(radius, thickness);
      
      Random random = new Random(1892L);
      double translation = (random.nextDouble() - 0.5) * 100.0;

      RigidBodyTransform transform = new RigidBodyTransform();
      
      transform.setTranslation(new Vector3D(translation, 0.0, 0.0));
      torus3d.setTransform(transform);
      testPointsInsideWhenOffsetBy(torus3d, translation, 0.0, 0.0);
      
      translation = (random.nextDouble() - 0.5) * 10.0;
      transform.setTranslation(new Vector3D(0.0, translation, 0.0));
      torus3d.setTransform(transform);
      testPointsInsideWhenOffsetBy(torus3d, 0.0, translation, 0.0);

      translation = (random.nextDouble() - 0.5) * 10.0;
//      System.out.println("Torus3dTest:testTranslatedPointOnOrInside:" + "0,0," + translation);
      transform.setTranslation(new Vector3D(0.0, 0.0, translation));
      torus3d.setTransform(transform);
      testPointsInsideWhenOffsetBy(torus3d, 0.0, 0.0, translation);

      translation = (random.nextDouble() - 0.5) * 10.0;
      double translationY = (random.nextDouble() - 0.5) * 10.0;
      double translationZ = (random.nextDouble() - 0.5) * 10.0;
//      System.out.println("Torus3dTest:testTranslatedPointOnOrInside:" + translation + "," + translationY + "," + translationZ);
      transform.setTranslation(new Vector3D(translation, translationY, translationZ));
      torus3d.setTransform(transform);
      testPointsInsideWhenOffsetBy(torus3d, translation, translationY, translationZ);
   }
   
   public void testPointsInsideWhenOffsetBy(Torus3d torus3d, double tx, double ty, double tz)
   {
      double radius = torus3d.getRadius();
      double thickness = torus3d.getThickness();
      
//      System.out.println("Testing points for offsets " + tx + "," + ty + "," + tz + " for torus : " + torus3d);
      // center point is false
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty, tz)));
      
      // left, right, top and bottom at the radius are true
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx+radius, ty, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx, ty+radius, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx-radius, ty, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx, ty-radius, tz)));
      
      // Z direction at the radius is false
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty, tz+radius)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty, tz-radius)));

      // on the left, right, top, and bottom outside surface edges
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx+radius+thickness, ty, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx, ty+radius+thickness, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx-(radius+thickness), ty, tz)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx, ty-(radius+thickness), tz)));
      
      // Z direction at the center is always false
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty, tz+radius+thickness)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty, tz-(radius+thickness))));

      // At radius in X or Y and thickness in Z should be true
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx+radius, ty, tz+thickness)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx-radius, ty, tz+thickness)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx+radius, ty, tz-thickness)));
      assertTrue(torus3d.isInsideOrOnSurface(new Point3D(tx-radius, ty, tz-thickness)));

      // left, right, top, bottom just beyond the surface
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx+radius+thickness+epsilon, ty, tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty+radius+thickness+epsilon, tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx-(radius+thickness+epsilon), ty, tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty-(radius+thickness+epsilon), tz)));

      // at radius, but Z just beyond the surface
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx+radius, ty, tz+thickness+epsilon)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx+radius, ty, tz-(thickness+epsilon))));

      // left, right, top, bottom just inside the inner ring of the surface
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx+radius-(thickness+epsilon), ty, tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty+radius-(thickness+epsilon), tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx-(radius-(thickness+epsilon)), ty, tz)));
      assertFalse(torus3d.isInsideOrOnSurface(new Point3D(tx, ty-(radius-(thickness+epsilon)), tz)));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIndependenceOfCopiedTransforms()
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(Math.PI / 6);
      Torus3d torus = new Torus3d(transform, 7.0, 2.0);
      
      Torus3d torusCopy = new Torus3d(torus);
      RigidBodyTransform transformAppliedOnlyToCopy = new RigidBodyTransform();
      transformAppliedOnlyToCopy.setRotationPitchAndZeroTranslation(Math.PI / 4);
      torusCopy.applyTransform(transformAppliedOnlyToCopy);
      assertFalse(torusCopy.getTransformUnsafe().equals(torus.getTransformUnsafe()));
      
      Torus3d torusCopyBySet = new Torus3d(5.0, 1.0);
      torusCopyBySet.set(torus);
      RigidBodyTransform transformAppliedOnlyToCopyBySet = new RigidBodyTransform();
      transformAppliedOnlyToCopyBySet.setRotationYawAndZeroTranslation(Math.PI / 5);
      torusCopyBySet.applyTransform(transformAppliedOnlyToCopyBySet);      
      assertFalse(torusCopyBySet.getTransformUnsafe().equals(torus.getTransformUnsafe()));
   }
}
