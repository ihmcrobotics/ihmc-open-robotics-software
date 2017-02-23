package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

public class BoundingBox2dTest
{
   double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMinPoint()
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      Point2D minPoint = new Point2D(boundingBox2d.getMinPoint());
      assertEquals(minPoint, lowerLeftPoint);

      minPoint = new Point2D();
      boundingBox2d.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMinPoint_2()
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint.getX(), lowerLeftPoint.getY(), upperRightPoint.getX(), upperRightPoint.getY());

      Point2D minPoint = new Point2D(boundingBox2d.getMinPoint());
      assertEquals(minPoint, lowerLeftPoint);

      minPoint = new Point2D();
      boundingBox2d.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMinPoint_3()
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d();
      boundingBox2d.set(lowerLeftPoint.getX(), lowerLeftPoint.getY(), upperRightPoint.getX(), upperRightPoint.getY());

      Point2D minPoint = new Point2D(boundingBox2d.getMinPoint());
      assertEquals(minPoint, lowerLeftPoint);

      minPoint = new Point2D();
      boundingBox2d.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
   public void testGetMinPoint_4()
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d();
      boundingBox2d.set(4.0, lowerLeftPoint.getY(), upperRightPoint.getX(), upperRightPoint.getY());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
   public void testGetMinPoint_5()
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d();
      boundingBox2d.set(lowerLeftPoint.getX(), 4.0, upperRightPoint.getX(), upperRightPoint.getY());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMaxPoint()
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      Point2D maxPoint = new Point2D(boundingBox2d.getMaxPoint());
      assertEquals(maxPoint, upperRightPoint);

      maxPoint = new Point2D();
      boundingBox2d.getMaxPoint(maxPoint);
      assertEquals(maxPoint, upperRightPoint);
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
         Point2D expectedCenterPoint = new Point2D(centerX, centerY);

         // create a random bounding box
         double randomX = random.nextDouble();
         double randomY = random.nextDouble();
         Point2D lowerLeftPoint = new Point2D(expectedCenterPoint.getX() - randomX, expectedCenterPoint.getY() - randomY);
         Point2D upperRightPoint = new Point2D(expectedCenterPoint.getX() + randomX, expectedCenterPoint.getY() + randomY);
         BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

         // determine center point
         Point2D actualCenterPoint = new Point2D();
         boundingBox2d.getCenterPointCopy(actualCenterPoint);
         assertEquals(expectedCenterPoint.getX(), actualCenterPoint.getX(), epsilon);
         assertEquals(expectedCenterPoint.getY(), actualCenterPoint.getY(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsFullyAbove()
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      assertTrue(boundingBox2d.isBoxAtOrAbove(-1.0));
      assertFalse(boundingBox2d.isBoxAtOrAbove(2.0));
      assertTrue(boundingBox2d.isBoxAtOrAbove(0.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsFullyBelow()
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      assertTrue(boundingBox2d.isBoxAtOrBelow(3.0));
      assertTrue(boundingBox2d.isBoxAtOrBelow(2.0));
      assertFalse(boundingBox2d.isBoxAtOrBelow(1.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsFullyLeft()
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      assertTrue(boundingBox2d.isBoxAtOrLeftOf(3.0));
      assertTrue(boundingBox2d.isBoxAtOrLeftOf(2.0));
      assertFalse(boundingBox2d.isBoxAtOrLeftOf(1.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsFullyRight() throws Exception
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      assertTrue(boundingBox2d.isBoxAtOrRightOf(-1.0));
      assertTrue(boundingBox2d.isBoxAtOrRightOf(0.0));
      assertFalse(boundingBox2d.isBoxAtOrRightOf(1.0));
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
         Point2D expectedCenterPoint = new Point2D(centerX, centerY);

         // create a random bounding box
         double randomX = random.nextDouble();
         double randomY = random.nextDouble();
         Point2D lowerLeftPoint = new Point2D(expectedCenterPoint.getX() - randomX, expectedCenterPoint.getY() - randomY);
         Point2D upperRightPoint = new Point2D(expectedCenterPoint.getX() + randomX, expectedCenterPoint.getY() + randomY);
         BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

         Point2D actualCenterPoint = new Point2D();
         boundingBox2d.getCenterPointCopy(actualCenterPoint);

         Point2D inPoint = new Point2D(centerX + (randomX * 0.9), centerY + (randomY * 0.9));
         assertTrue(boundingBox2d.isInside(inPoint));
         Point2D onPoint = new Point2D(centerX + randomX, centerY + randomY);
         assertTrue(boundingBox2d.isInside(onPoint));
         int randomInt = random.nextInt(4);
         double xOffset = 0.0;
         double yOffset = 0.0;
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

         Point2D outPoint = new Point2D(centerX + xOffset, centerY + yOffset);
         assertFalse(boundingBox2d.isInside(outPoint));
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersects()
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2dA = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      // identical boxes
      lowerLeftPoint = new Point2D(0.0, 0.0);
      upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2dB = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertTrue(boundingBox2dA.intersects(boundingBox2dB));

      // partial overlap
      lowerLeftPoint = new Point2D(1.0, 1.0);
      upperRightPoint = new Point2D(3.0, 3.0);
      BoundingBox2d boundingBox2dC = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertTrue(boundingBox2dA.intersects(boundingBox2dC));

      // contained within
      lowerLeftPoint = new Point2D(0.5, 0.5);
      upperRightPoint = new Point2D(1.5, 1.5);
      BoundingBox2d boundingBox2dD = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertTrue(boundingBox2dA.intersects(boundingBox2dD));

      // edge similarity
      lowerLeftPoint = new Point2D(0.0, 2.0);
      upperRightPoint = new Point2D(1.0, 4.0);
      BoundingBox2d boundingBox2dE = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertTrue(boundingBox2dA.intersects(boundingBox2dE));

      // corner similarity
      lowerLeftPoint = new Point2D(2.0, 2.0);
      upperRightPoint = new Point2D(3.0, 3.0);
      BoundingBox2d boundingBox2dF = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertTrue(boundingBox2dA.intersects(boundingBox2dF));

      // no overlap
      lowerLeftPoint = new Point2D(5.0, 5.0);
      upperRightPoint = new Point2D(7.0, 7.0);
      BoundingBox2d boundingBox2dG = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertFalse(boundingBox2dA.intersects(boundingBox2dG));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructors()
   {
      Point2D lowerLeftPoint = new Point2D(0.0, 0.0);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2dA = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      // identical boxes
      double[] min = {0.0, 0.0};
      double[] max = {2.0, 2.0};
      BoundingBox2d boundingBox2dB = new BoundingBox2d(min, max);
      assertTrue(boundingBox2dA.intersects(boundingBox2dB));

      Point2D expectedCenterPoint = new Point2D();
      boundingBox2dA.getCenterPointCopy(expectedCenterPoint);
      Point2D actualCenterPoint = new Point2D();
      boundingBox2dB.getCenterPointCopy(actualCenterPoint);
      assertEquals(expectedCenterPoint.getX(), actualCenterPoint.getX(), epsilon);
      assertEquals(expectedCenterPoint.getY(), actualCenterPoint.getY(), epsilon);

      // identical boxes
      BoundingBox2d boundingBox2dC = new BoundingBox2d(boundingBox2dA);
      assertTrue(boundingBox2dA.intersects(boundingBox2dC));

      expectedCenterPoint = new Point2D();
      boundingBox2dA.getCenterPointCopy(expectedCenterPoint);
      actualCenterPoint = new Point2D();
      boundingBox2dC.getCenterPointCopy(actualCenterPoint);
      assertEquals(expectedCenterPoint.getX(), actualCenterPoint.getX(), epsilon);
      assertEquals(expectedCenterPoint.getY(), actualCenterPoint.getY(), epsilon);
   }

   // @todo test things like passing in null in the constructor
// public void testAnomalies()
// {
//    Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
//    Point2d upperRightPoint = null;
//    BoundingBox2d boundingBox2dA = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
// }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetPointGivenParameters()
   {
      Random random = new Random(125L);
      Point2D lowerLeftPoint = new Point2D(random.nextDouble(), random.nextDouble());
      Point2D upperRightPoint = new Point2D(lowerLeftPoint);
      upperRightPoint.add(new Vector2D(random.nextDouble(), random.nextDouble()));
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      Point2D point = new Point2D();

      boundingBox2d.getPointGivenParameters(point, 0.0, 0.0);
      assertTrue(lowerLeftPoint.epsilonEquals(point, 0.0));

      boundingBox2d.getPointGivenParameters(point, 1.0, 1.0);
      assertTrue(upperRightPoint.epsilonEquals(point, 0.0));

      boundingBox2d.getPointGivenParameters(point, 1.0, 0.0);
      assertEquals(upperRightPoint.getX(), point.getX(), 0.0);
      assertEquals(lowerLeftPoint.getY(), point.getY(), 0.0);

      boundingBox2d.getPointGivenParameters(point, 0.0, 1.0);
      assertEquals(lowerLeftPoint.getX(), point.getX(), 0.0);
      assertEquals(upperRightPoint.getY(), point.getY(), 0.0);

      Point2D average = new Point2D();
      average.interpolate(lowerLeftPoint, upperRightPoint, 0.5);
      boundingBox2d.getPointGivenParameters(point, 0.5, 0.5);
      assertTrue(average.epsilonEquals(point, 0.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSet()
   {
      Point2D lowerLeftPoint = new Point2D(0.0123, 0.0456);
      Point2D upperRightPoint = new Point2D(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d();
      boundingBox2d.set(lowerLeftPoint.getX(), lowerLeftPoint.getY(), upperRightPoint.getX(), upperRightPoint.getY());

      BoundingBox2d boundingBoxCopy = new BoundingBox2d();
      boundingBoxCopy.set(boundingBox2d);

      Point2D minPoint = new Point2D();
      boundingBoxCopy.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);

      Point2D maxPoint = new Point2D();
      boundingBoxCopy.getMaxPoint(maxPoint);
      assertEquals(maxPoint, upperRightPoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetDiagonalDistanceSquared()
   {
      Point2D lowerLeftPoint = new Point2D(1.0, 7.0);
      Point2D upperRightPoint = new Point2D(4.0, 11.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d();
      boundingBox2d.set(lowerLeftPoint.getX(), lowerLeftPoint.getY(), upperRightPoint.getX(), upperRightPoint.getY());

      assertEquals(25.0, boundingBox2d.getDiagonalLengthSquared(), 1e-7);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(BoundingBox2d.class, BoundingBox2dTest.class);
   }
}
