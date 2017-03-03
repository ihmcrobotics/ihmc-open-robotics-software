package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class BoundingBox3dTest
{
   double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCreateUsingCenterAndPlusMinusVector()
   {
      BoundingBox3d box = BoundingBox3d.createUsingCenterAndPlusMinusVector(new Point3D(0.0, 0.0, 0.0), new Vector3D(1, 1, 1));

      assertEquals(box.getXMin(), -1.0, 1e-7);
      assertEquals(box.getXMax(), 1.0, 1e-7);
      assertEquals(box.getYMin(), -1.0, 1e-7);
      assertEquals(box.getYMax(), 1.0, 1e-7);
      assertEquals(box.getZMin(), -1.0, 1e-7);
      assertEquals(box.getZMax(), 1.0, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMinPoint()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 0.1, 0.6);
      Point3D upperRightPoint = new Point3D(2.0, 2.1, 5.6);
      BoundingBox3d boundingBox3d = new BoundingBox3d(lowerLeftPoint, upperRightPoint);

      Point3D minPoint = new Point3D();
      boundingBox3d.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);

      minPoint = new Point3D();
      boundingBox3d.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);

      double epsilon = 1e-14;
      assertEquals(minPoint.getX(), lowerLeftPoint.getX(), epsilon);
      assertEquals(minPoint.getY(), lowerLeftPoint.getY(), epsilon);
      assertEquals(minPoint.getZ(), lowerLeftPoint.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMinPoint_2()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 0.1, 0.6);
      Point3D upperRightPoint = new Point3D(2.0, 2.1, 5.6);
      BoundingBox3d boundingBox3d = new BoundingBox3d(lowerLeftPoint.getX(), lowerLeftPoint.getY(), lowerLeftPoint.getZ(), upperRightPoint.getX(), upperRightPoint.getY(),
                                       upperRightPoint.getZ());

      Point3D minPoint = new Point3D();
      boundingBox3d.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);

      minPoint = new Point3D();
      boundingBox3d.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);

      double epsilon = 1e-14;
      assertEquals(minPoint.getX(), lowerLeftPoint.getX(), epsilon);
      assertEquals(minPoint.getY(), lowerLeftPoint.getY(), epsilon);
      assertEquals(minPoint.getZ(), lowerLeftPoint.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testVerifyBounds()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 0.1, 0.6);
      Point3D upperRightPoint = new Point3D(2.0, 2.1, 5.6);
      BoundingBox3d boundingBox3d = new BoundingBox3d(4.0, lowerLeftPoint.getY(), lowerLeftPoint.getZ(), upperRightPoint.getX(), upperRightPoint.getY(), upperRightPoint.getZ());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testVerifyBounds_2()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 0.1, 0.6);
      Point3D upperRightPoint = new Point3D(2.0, 2.1, 5.6);
      BoundingBox3d boundingBox3d = new BoundingBox3d(lowerLeftPoint.getX(), 4.0, lowerLeftPoint.getZ(), upperRightPoint.getX(), upperRightPoint.getY(), upperRightPoint.getZ());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected = RuntimeException.class)
   public void testVerifyBounds_3()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 0.1, 0.6);
      Point3D upperRightPoint = new Point3D(2.0, 2.1, 5.6);
      BoundingBox3d boundingBox3d = new BoundingBox3d(lowerLeftPoint.getX(), lowerLeftPoint.getY(), 10.0, upperRightPoint.getX(), upperRightPoint.getY(), upperRightPoint.getZ());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMin()
   {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
         Point3D lowerLeftPoint = new Point3D(rand.nextDouble() * 1000, rand.nextDouble() * 1000, rand.nextDouble() * 1000);
         Point3D upperRightPoint = new Point3D(lowerLeftPoint.getX() + 1.0, lowerLeftPoint.getY() + 1.0, lowerLeftPoint.getZ() + 1.0);
         BoundingBox3d boundingBox3d = new BoundingBox3d(lowerLeftPoint.getX(), lowerLeftPoint.getY(), lowerLeftPoint.getZ(), upperRightPoint.getX(), upperRightPoint.getY(),
                                          upperRightPoint.getZ());
         assertEquals(boundingBox3d.getXMin(), lowerLeftPoint.getX(), 1e-7);
         assertEquals(boundingBox3d.getYMin(), lowerLeftPoint.getY(), 1e-7);
         assertEquals(boundingBox3d.getZMin(), lowerLeftPoint.getZ(), 1e-7);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMax()
   {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
         Point3D lowerLeftPoint = new Point3D(rand.nextDouble() * 1000, rand.nextDouble() * 1000, rand.nextDouble() * 1000);
         Point3D upperRightPoint = new Point3D(lowerLeftPoint.getX() + 1.0, lowerLeftPoint.getY() + 1.0, lowerLeftPoint.getZ() + 1.0);
         BoundingBox3d boundingBox3d = new BoundingBox3d(lowerLeftPoint.getX(), lowerLeftPoint.getY(), lowerLeftPoint.getZ(), upperRightPoint.getX(), upperRightPoint.getY(),
                                          upperRightPoint.getZ());
         assertEquals(boundingBox3d.getXMax(), upperRightPoint.getX(), 1e-7);
         assertEquals(boundingBox3d.getYMax(), upperRightPoint.getY(), 1e-7);
         assertEquals(boundingBox3d.getZMax(), upperRightPoint.getZ(), 1e-7);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMaxPoint()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 0.1, 0.6);
      Point3D upperRightPoint = new Point3D(2.0, 2.1, 5.6);
      BoundingBox3d BoundingBox3d = new BoundingBox3d(lowerLeftPoint, upperRightPoint);

      Point3D maxPoint = new Point3D();
      BoundingBox3d.getMaxPoint(maxPoint);
      assertEquals(maxPoint, upperRightPoint);

      maxPoint = new Point3D();
      BoundingBox3d.getMaxPoint(maxPoint);
      assertEquals(maxPoint, upperRightPoint);

      double epsilon = 1e-14;
      assertEquals(maxPoint.getX(), upperRightPoint.getX(), epsilon);
      assertEquals(maxPoint.getY(), upperRightPoint.getY(), epsilon);
      assertEquals(maxPoint.getZ(), upperRightPoint.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetCenterPointCopy()
   {
      Random random = new Random(7);
      for (int i = 0; i < 100; i++)
      {
         // create a random center point
         double centerX = random.nextDouble();
         double centerY = random.nextDouble();
         double centerZ = random.nextDouble();
         Point3D expectedCenterPoint = new Point3D(centerX, centerY, centerZ);

         // create a random bounding box
         double randomX = random.nextDouble();
         double randomY = random.nextDouble();
         double randomZ = random.nextDouble();
         Point3D lowerLeftPoint = new Point3D(expectedCenterPoint.getX() - randomX, expectedCenterPoint.getY() - randomY, expectedCenterPoint.getZ() - randomZ);
         Point3D upperRightPoint = new Point3D(expectedCenterPoint.getX() + randomX, expectedCenterPoint.getY() + randomY,
                                      expectedCenterPoint.getZ() + randomZ);
         BoundingBox3d BoundingBox3d = new BoundingBox3d(lowerLeftPoint, upperRightPoint);

         // determine center point
         Point3D actualCenterPoint = new Point3D();
         BoundingBox3d.getCenterPoint(actualCenterPoint);
         assertEquals(expectedCenterPoint.getX(), actualCenterPoint.getX(), epsilon);
         assertEquals(expectedCenterPoint.getY(), actualCenterPoint.getY(), epsilon);
         assertEquals(expectedCenterPoint.getZ(), actualCenterPoint.getZ(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsAtOrAbove()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 1.1, 2.7);
      Point3D upperRightPoint = new Point3D(0.5, 2.3, 9.99);
      BoundingBox3d BoundingBox3d = new BoundingBox3d(lowerLeftPoint, upperRightPoint);

      assertTrue(BoundingBox3d.isBoxAtOrAbove(-1.0));
      assertTrue(BoundingBox3d.isBoxAtOrAbove(2.7));
      assertFalse(BoundingBox3d.isBoxAtOrAbove(10.0));
      assertFalse(BoundingBox3d.isBoxAtOrAbove(5.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsAtOrBelow()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 1.1, 2.7);
      Point3D upperRightPoint = new Point3D(0.5, 2.3, 9.99);
      BoundingBox3d BoundingBox3d = new BoundingBox3d(lowerLeftPoint, upperRightPoint);

      assertTrue(BoundingBox3d.isBoxAtOrBelow(12.0));
      assertTrue(BoundingBox3d.isBoxAtOrBelow(9.99));
      assertFalse(BoundingBox3d.isBoxAtOrBelow(5.6));
      assertFalse(BoundingBox3d.isBoxAtOrBelow(2.69));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsAtOrLeftOf()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 1.1, 2.7);
      Point3D upperRightPoint = new Point3D(0.5, 2.3, 9.99);
      BoundingBox3d BoundingBox3d = new BoundingBox3d(lowerLeftPoint, upperRightPoint);

      assertTrue(BoundingBox3d.isBoxAtOrLeftOf(1.0));
      assertTrue(BoundingBox3d.isBoxAtOrLeftOf(1.1));
      assertFalse(BoundingBox3d.isBoxAtOrLeftOf(1.5));
      assertFalse(BoundingBox3d.isBoxAtOrLeftOf(3.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsAtOrRightOf()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 1.1, 2.7);
      Point3D upperRightPoint = new Point3D(0.5, 2.3, 9.99);
      BoundingBox3d BoundingBox3d = new BoundingBox3d(lowerLeftPoint, upperRightPoint);

      assertTrue(BoundingBox3d.isBoxAtOrRightOf(2.3));
      assertTrue(BoundingBox3d.isBoxAtOrRightOf(2.6));
      assertFalse(BoundingBox3d.isBoxAtOrRightOf(2.2));
      assertFalse(BoundingBox3d.isBoxAtOrRightOf(1.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsBoxAtOrInFrontOf()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 1.1, 2.7);
      Point3D upperRightPoint = new Point3D(0.5, 2.3, 9.99);
      BoundingBox3d BoundingBox3d = new BoundingBox3d(lowerLeftPoint, upperRightPoint);

      assertTrue(BoundingBox3d.isBoxAtOrInFrontOf(0.0));
      assertTrue(BoundingBox3d.isBoxAtOrInFrontOf(-0.5));
      assertFalse(BoundingBox3d.isBoxAtOrInFrontOf(0.2));
      assertFalse(BoundingBox3d.isBoxAtOrInFrontOf(0.6));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsBoxAtOrBehind()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 1.1, 2.7);
      Point3D upperRightPoint = new Point3D(0.5, 2.3, 9.99);
      BoundingBox3d BoundingBox3d = new BoundingBox3d(lowerLeftPoint, upperRightPoint);

      assertTrue(BoundingBox3d.isBoxAtOrBehind(0.7));
      assertTrue(BoundingBox3d.isBoxAtOrBehind(0.5));
      assertFalse(BoundingBox3d.isBoxAtOrBehind(0.3));
      assertFalse(BoundingBox3d.isBoxAtOrBehind(-0.2));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsInside()
   {
      Random random = new Random(7);
      for (int i = 0; i < 100; i++)
      {
         // create a random center point
         double centerX = random.nextDouble();
         double centerY = random.nextDouble();
         double centerZ = random.nextDouble();
         Point3D expectedCenterPoint = new Point3D(centerX, centerY, centerZ);

         // create a random bounding box
         double randomX = random.nextDouble();
         double randomY = random.nextDouble();
         double randomZ = random.nextDouble();
         Point3D lowerLeftPoint = new Point3D(expectedCenterPoint.getX() - randomX, expectedCenterPoint.getY() - randomY, expectedCenterPoint.getZ() - randomZ);
         Point3D upperRightPoint = new Point3D(expectedCenterPoint.getX() + randomX, expectedCenterPoint.getY() + randomY,
                                      expectedCenterPoint.getZ() + randomZ);
         BoundingBox3d BoundingBox3d = new BoundingBox3d(lowerLeftPoint, upperRightPoint);

         Point3D actualCenterPoint = new Point3D();
         BoundingBox3d.getCenterPoint(actualCenterPoint);

         Point3D inPoint = new Point3D(centerX + (randomX * 0.9), centerY + (randomY * 0.9), centerZ + (randomZ * 0.9));
         assertTrue("In Point not inside of bounding box: " + inPoint + "\nBox: " + BoundingBox3d, BoundingBox3d.isInside(inPoint));
         Point3D onPoint = new Point3D(centerX + randomX, centerY + randomY, centerZ + randomZ);
         assertTrue("On Point not on bounding box: " + onPoint + "\nBox: " + BoundingBox3d, BoundingBox3d.isInside(onPoint));

         int randomInt = random.nextInt(6);
         double xOffset = 0.0;
         double yOffset = 0.0;
         double zOffset = 0.0;

         if (randomInt == 0)
         {
            xOffset = randomX * 1.01;
         }
         else if (randomInt == 1)
         {
            xOffset = randomX * -1.01;
         }
         else if (randomInt == 2)
         {
            yOffset = randomY * 1.01;
         }
         else if (randomInt == 3)
         {
            yOffset = randomY * -1.01;
         }
         else if (randomInt == 4)
         {
            zOffset = randomZ * 1.01;
         }
         else if (randomInt == 5)
         {
            zOffset = randomZ * -1.01;
         }

         Point3D outPoint = new Point3D(centerX + xOffset, centerY + yOffset, centerZ + zOffset);
         assertFalse(BoundingBox3d.isInside(outPoint));
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPointIntersects()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 0.0, 0.0);
      Point3D upperRightPoint = new Point3D(2.0, 2.0, 2.0);
      BoundingBox3d BoundingBox3dA = new BoundingBox3d(lowerLeftPoint, upperRightPoint);
      assertTrue(BoundingBox3dA.intersects(BoundingBox3dA));

      Point3D start, end;

      // line inside box
      start = new Point3D(1.0, 1.0, 1.0);
      end = new Point3D(1.0, 1.0, 1.0);
      assertTrue(BoundingBox3dA.intersects(start, end));

      // line outside box
      start = new Point3D(3.0, 1.0, 1.0);
      end = new Point3D(4.0, 1.0, 1.0);
      assertFalse(BoundingBox3dA.intersects(start, end));

      // line through box
      start = new Point3D(-1.0, 1.0, 1.0);
      end = new Point3D(3.0, 1.0, 1.0);
      assertTrue(BoundingBox3dA.intersects(start, end));

      // reverse line through box
      start = new Point3D(3.0, 1.0, 1.0);
      end = new Point3D(-1.0, 1.0, 1.0);
      assertTrue(BoundingBox3dA.intersects(start, end));

      // reverse line through box
      start = new Point3D(3.0, 1.0, 3.0);
      end = new Point3D(-1.0, 1.0, -1.0);
      assertTrue(BoundingBox3dA.intersects(start, end));

      // one point inside
      start = new Point3D(1.0, 1.0, 1.0);
      end = new Point3D(-1.0, 1.0, 1.0);
      assertTrue(BoundingBox3dA.intersects(start, end));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testBoxIntersects()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 0.0, 0.0);
      Point3D upperRightPoint = new Point3D(2.0, 2.0, 2.0);
      BoundingBox3d BoundingBox3dA = new BoundingBox3d(lowerLeftPoint, upperRightPoint);
      assertTrue(BoundingBox3dA.intersects(BoundingBox3dA));

      // identical boxes
      lowerLeftPoint = new Point3D(0.0, 1.0, 1.2);
      upperRightPoint = new Point3D(2.1, 2.3, 5.5);
      BoundingBox3d BoundingBox3dB = new BoundingBox3d(lowerLeftPoint, upperRightPoint);
      assertTrue(BoundingBox3dA.intersects(BoundingBox3dB));

      // partial overlap
      lowerLeftPoint = new Point3D(1.0, 1.0, 1.0);
      upperRightPoint = new Point3D(3.0, 3.0, 3.0);
      BoundingBox3d BoundingBox3dC = new BoundingBox3d(lowerLeftPoint, upperRightPoint);
      assertTrue(BoundingBox3dA.intersects(BoundingBox3dC));

      // contained within
      lowerLeftPoint = new Point3D(0.5, 0.5, 0.5);
      upperRightPoint = new Point3D(1.5, 1.5, 1.5);
      BoundingBox3d BoundingBox3dD = new BoundingBox3d(lowerLeftPoint, upperRightPoint);
      assertTrue(BoundingBox3dA.intersects(BoundingBox3dD));

      // edge similarity
      lowerLeftPoint = new Point3D(0.0, 2.0, 2.0);
      upperRightPoint = new Point3D(1.0, 4.0, 4.0);
      BoundingBox3d BoundingBox3dE = new BoundingBox3d(lowerLeftPoint, upperRightPoint);
      assertTrue(BoundingBox3dA.intersects(BoundingBox3dE));

      // corner similarity
      lowerLeftPoint = new Point3D(2.0, 2.0, 2.0);
      upperRightPoint = new Point3D(3.0, 3.0, 3.0);
      BoundingBox3d BoundingBox3dF = new BoundingBox3d(lowerLeftPoint, upperRightPoint);
      assertTrue(BoundingBox3dA.intersects(BoundingBox3dF));

      // no overlap
      lowerLeftPoint = new Point3D(5.0, 5.0, 5.0);
      upperRightPoint = new Point3D(7.0, 7.0, 7.0);
      BoundingBox3d BoundingBox3dG = new BoundingBox3d(lowerLeftPoint, upperRightPoint);
      assertFalse(BoundingBox3dA.intersects(BoundingBox3dG));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructors()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 0.3, 0.9);
      Point3D upperRightPoint = new Point3D(2.0, 3.0, 6.0);
      BoundingBox3d BoundingBox3dA = new BoundingBox3d(lowerLeftPoint, upperRightPoint);

      // identical boxes
      double[] min = {0.0, 0.3, 0.9};
      double[] max = {2.0, 3.0, 6.0};
      BoundingBox3d BoundingBox3dB = new BoundingBox3d(min, max);
      assertTrue(BoundingBox3dA.intersects(BoundingBox3dB));

      Point3D expectedCenterPoint = new Point3D();
      BoundingBox3dA.getCenterPoint(expectedCenterPoint);
      Point3D actualCenterPoint = new Point3D();
      BoundingBox3dB.getCenterPoint(actualCenterPoint);
      assertEquals(expectedCenterPoint.getX(), actualCenterPoint.getX(), epsilon);
      assertEquals(expectedCenterPoint.getY(), actualCenterPoint.getY(), epsilon);
      assertEquals(expectedCenterPoint.getZ(), actualCenterPoint.getZ(), epsilon);

      // identical boxes
      BoundingBox3d BoundingBox3dC = new BoundingBox3d(BoundingBox3dA);
      assertTrue(BoundingBox3dA.intersects(BoundingBox3dC));

      expectedCenterPoint = new Point3D();
      BoundingBox3dA.getCenterPoint(expectedCenterPoint);
      actualCenterPoint = new Point3D();
      BoundingBox3dC.getCenterPoint(actualCenterPoint);
      assertEquals(expectedCenterPoint.getX(), actualCenterPoint.getX(), epsilon);
      assertEquals(expectedCenterPoint.getY(), actualCenterPoint.getY(), epsilon);
      assertEquals(expectedCenterPoint.getZ(), actualCenterPoint.getZ(), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testUnion()
   {
      Point3D lowerLeftPointA = new Point3D(0.0, 1.0, 2.0);
      Point3D upperRightPointA = new Point3D(3.0, 4.0, 5.0);
      BoundingBox3d boundingBoxA = new BoundingBox3d(lowerLeftPointA, upperRightPointA);

      Point3D lowerLeftPointB = new Point3D(6.0, 7.0, 8.0);
      Point3D upperRightPointB = new Point3D(9.0, 10.0, 11.0);
      BoundingBox3d boundingBoxB = new BoundingBox3d(lowerLeftPointB, upperRightPointB);

      BoundingBox3d union = BoundingBox3d.union(boundingBoxA, boundingBoxB);
      double epsilon = 1e-14;

      Point3D minPoint = new Point3D();
      union.getMinPoint(minPoint);
      Point3D maxPoint = new Point3D();
      union.getMaxPoint(maxPoint);

      EuclidCoreTestTools.assertTuple3DEquals(lowerLeftPointA, minPoint, epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(upperRightPointB, maxPoint, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testUpdateToIncludePointThatIsAlreadyInsideBoundingBox()
   {
      Point3D originalMinPoint = new Point3D(0.0, 0.0, 0.0);
      Point3D originalMaxPoint = new Point3D(1.0, 1.0, 1.0);
      BoundingBox3d boundingBox = new BoundingBox3d(originalMinPoint, originalMaxPoint);

      boundingBox.updateToIncludePoint(0.5, 0.5, 0.5);

      Point3D updatedMinPointToPack = new Point3D();
      Point3D updatedMaxPointToPack = new Point3D();

      boundingBox.getMinPoint(updatedMinPointToPack);
      boundingBox.getMaxPoint(updatedMaxPointToPack);

      assertEquals(originalMinPoint, updatedMinPointToPack);
      assertEquals(originalMaxPoint, updatedMaxPointToPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testUpdateToIncludeNewMaxPointOutsideOfBoundingBox()
   {
      Point3D originalMinPoint = new Point3D(0.0, 0.0, 0.0);
      Point3D originalMaxPoint = new Point3D(1.0, 1.0, 1.0);
      BoundingBox3d boundingBox = new BoundingBox3d(originalMinPoint, originalMaxPoint);

      Point3D newMaxPoint = new Point3D(1.5, 1.5, 1.5);
      boundingBox.updateToIncludePoint(newMaxPoint);

      Point3D updatedMinPointToPack = new Point3D();
      Point3D updatedMaxPointToPack = new Point3D();

      boundingBox.getMinPoint(updatedMinPointToPack);
      boundingBox.getMaxPoint(updatedMaxPointToPack);

      assertEquals(originalMinPoint, updatedMinPointToPack);
      assertEquals(newMaxPoint, updatedMaxPointToPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testUpdateToIncludeNewMinPointOutsideOfBoundingBox()
   {
      Point3D originalMinPoint = new Point3D(0.0, 0.0, 0.0);
      Point3D originalMaxPoint = new Point3D(1.0, 1.0, 1.0);
      BoundingBox3d boundingBox = new BoundingBox3d(originalMinPoint, originalMaxPoint);

      Point3D newMinPoint = new Point3D(-1.5, -1.5, -1.5);
      boundingBox.updateToIncludePoint(newMinPoint);

      Point3D updatedMinPointToPack = new Point3D();
      Point3D updatedMaxPointToPack = new Point3D();

      boundingBox.getMinPoint(updatedMinPointToPack);
      boundingBox.getMaxPoint(updatedMaxPointToPack);

      assertEquals(newMinPoint, updatedMinPointToPack);
      assertEquals(originalMaxPoint, updatedMaxPointToPack);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetEpsilonToGrowWithNegativeArgument()
   {
      Point3D originalMinPoint = new Point3D(0.0, 0.0, 0.0);
      Point3D originalMaxPoint = new Point3D(1.0, 1.0, 1.0);
      BoundingBox3d boundingBox = new BoundingBox3d(originalMinPoint, originalMaxPoint);

      try
      {
         boundingBox.setEpsilonToGrow(-1.0);
      }
      catch (Throwable e)
      {
         return;
      }

      fail("Setting epsilonToGrow with negative argument did not result in a runtime exception!");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetEpsilonToShrinkWithPositiveArgument()
   {
      Point3D originalMinPoint = new Point3D(0.0, 0.0, 0.0);
      Point3D originalMaxPoint = new Point3D(1.0, 1.0, 1.0);
      BoundingBox3d boundingBox = new BoundingBox3d(originalMinPoint, originalMaxPoint);

      try
      {
         boundingBox.setEpsilonToShrink(1.0);
      }
      catch (Throwable e)
      {
         return;
      }

      fail("Setting epsilonToShrink with positive argument did not result in a runtime exception!");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetEpsilonToShrink()
   {
      Random random = new Random(1976L);

      Point3D originalMinPoint = new Point3D(0.0, 0.0, 0.0);
      Point3D originalMaxPoint = new Point3D(1.0, 1.0, 1.0);
      BoundingBox3d boundingBox = new BoundingBox3d(originalMinPoint, originalMaxPoint);

      for(int i = 0; i < 100000; i++)
      {
         double epsilonToShrink = RandomNumbers.nextDouble(random, -1e-16, -1e-4);
         boundingBox.setEpsilonToShrink(epsilonToShrink);
         assertFalse(boundingBox.isInside(originalMaxPoint));
         assertFalse(boundingBox.isInside(originalMinPoint));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetEpsilonToGrow()
   {
      Random random = new Random(1976L);

      Point3D originalMinPoint = new Point3D(0.0, 0.0, 0.0);
      Point3D originalMaxPoint = new Point3D(1.0, 1.0, 1.0);
      BoundingBox3d boundingBox = new BoundingBox3d(originalMinPoint, originalMaxPoint);

      for(int i = 0; i < 100000; i++)
      {
         double epsilonToShrink = RandomNumbers.nextDouble(random, 1e4, 1e16);
         boundingBox.setEpsilonToGrow(epsilonToShrink);
         assertTrue(boundingBox.isInside(originalMaxPoint));
         assertTrue(boundingBox.isInside(originalMinPoint));
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testToString()
   {
      Point3D lowerLeftPoint = new Point3D(0.0, 1.0, 2.0);
      Point3D upperRightPoint = new Point3D(3.0, 4.0, 5.0);
      BoundingBox3d BoundingBox3dA = new BoundingBox3d(lowerLeftPoint, upperRightPoint);
      String expectedString = "BoundingBox3d{" + "minPoint=" + lowerLeftPoint + " , maxPoint=" + upperRightPoint + '}';
      assertEquals(expectedString, BoundingBox3dA.toString());
   }
}
