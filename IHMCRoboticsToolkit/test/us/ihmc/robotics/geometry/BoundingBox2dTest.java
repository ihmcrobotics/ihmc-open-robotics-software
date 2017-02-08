package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.MutationTestingTools;

public class BoundingBox2dTest
{
   double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMinPoint()
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      Point2d minPoint = boundingBox2d.getMinPoint();
      assertEquals(minPoint, lowerLeftPoint);

      minPoint = new Point2d();
      boundingBox2d.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMinPoint_2()
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint.getX(), lowerLeftPoint.getY(), upperRightPoint.getX(), upperRightPoint.getY());

      Point2d minPoint = boundingBox2d.getMinPoint();
      assertEquals(minPoint, lowerLeftPoint);

      minPoint = new Point2d();
      boundingBox2d.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMinPoint_3()
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d();
      boundingBox2d.set(lowerLeftPoint.getX(), lowerLeftPoint.getY(), upperRightPoint.getX(), upperRightPoint.getY());

      Point2d minPoint = boundingBox2d.getMinPoint();
      assertEquals(minPoint, lowerLeftPoint);

      minPoint = new Point2d();
      boundingBox2d.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
   public void testGetMinPoint_4()
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d();
      boundingBox2d.set(4.0, lowerLeftPoint.getY(), upperRightPoint.getX(), upperRightPoint.getY());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
   public void testGetMinPoint_5()
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d();
      boundingBox2d.set(lowerLeftPoint.getX(), 4.0, upperRightPoint.getX(), upperRightPoint.getY());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetMaxPoint()
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      Point2d maxPoint = boundingBox2d.getMaxPoint();
      assertEquals(maxPoint, upperRightPoint);

      maxPoint = new Point2d();
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
         Point2d expectedCenterPoint = new Point2d(centerX, centerY);

         // create a random bounding box
         double randomX = random.nextDouble();
         double randomY = random.nextDouble();
         Point2d lowerLeftPoint = new Point2d(expectedCenterPoint.getX() - randomX, expectedCenterPoint.getY() - randomY);
         Point2d upperRightPoint = new Point2d(expectedCenterPoint.getX() + randomX, expectedCenterPoint.getY() + randomY);
         BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

         // determine center point
         Point2d actualCenterPoint = new Point2d();
         boundingBox2d.getCenterPointCopy(actualCenterPoint);
         assertEquals(expectedCenterPoint.getX(), actualCenterPoint.getX(), epsilon);
         assertEquals(expectedCenterPoint.getY(), actualCenterPoint.getY(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsFullyAbove()
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      assertTrue(boundingBox2d.isBoxAtOrAbove(-1.0));
      assertFalse(boundingBox2d.isBoxAtOrAbove(2.0));
      assertTrue(boundingBox2d.isBoxAtOrAbove(0.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsFullyBelow()
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      assertTrue(boundingBox2d.isBoxAtOrBelow(3.0));
      assertTrue(boundingBox2d.isBoxAtOrBelow(2.0));
      assertFalse(boundingBox2d.isBoxAtOrBelow(1.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsFullyLeft()
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      assertTrue(boundingBox2d.isBoxAtOrLeftOf(3.0));
      assertTrue(boundingBox2d.isBoxAtOrLeftOf(2.0));
      assertFalse(boundingBox2d.isBoxAtOrLeftOf(1.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsFullyRight() throws Exception
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
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
         Point2d expectedCenterPoint = new Point2d(centerX, centerY);

         // create a random bounding box
         double randomX = random.nextDouble();
         double randomY = random.nextDouble();
         Point2d lowerLeftPoint = new Point2d(expectedCenterPoint.getX() - randomX, expectedCenterPoint.getY() - randomY);
         Point2d upperRightPoint = new Point2d(expectedCenterPoint.getX() + randomX, expectedCenterPoint.getY() + randomY);
         BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

         Point2d actualCenterPoint = new Point2d();
         boundingBox2d.getCenterPointCopy(actualCenterPoint);

         Point2d inPoint = new Point2d(centerX + (randomX * 0.9), centerY + (randomY * 0.9));
         assertTrue(boundingBox2d.isInside(inPoint));
         Point2d onPoint = new Point2d(centerX + randomX, centerY + randomY);
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

         Point2d outPoint = new Point2d(centerX + xOffset, centerY + yOffset);
         assertFalse(boundingBox2d.isInside(outPoint));
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersects()
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2dA = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      // identical boxes
      lowerLeftPoint = new Point2d(0.0, 0.0);
      upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2dB = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertTrue(boundingBox2dA.intersects(boundingBox2dB));

      // partial overlap
      lowerLeftPoint = new Point2d(1.0, 1.0);
      upperRightPoint = new Point2d(3.0, 3.0);
      BoundingBox2d boundingBox2dC = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertTrue(boundingBox2dA.intersects(boundingBox2dC));

      // contained within
      lowerLeftPoint = new Point2d(0.5, 0.5);
      upperRightPoint = new Point2d(1.5, 1.5);
      BoundingBox2d boundingBox2dD = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertTrue(boundingBox2dA.intersects(boundingBox2dD));

      // edge similarity
      lowerLeftPoint = new Point2d(0.0, 2.0);
      upperRightPoint = new Point2d(1.0, 4.0);
      BoundingBox2d boundingBox2dE = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertTrue(boundingBox2dA.intersects(boundingBox2dE));

      // corner similarity
      lowerLeftPoint = new Point2d(2.0, 2.0);
      upperRightPoint = new Point2d(3.0, 3.0);
      BoundingBox2d boundingBox2dF = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertTrue(boundingBox2dA.intersects(boundingBox2dF));

      // no overlap
      lowerLeftPoint = new Point2d(5.0, 5.0);
      upperRightPoint = new Point2d(7.0, 7.0);
      BoundingBox2d boundingBox2dG = new BoundingBox2d(lowerLeftPoint, upperRightPoint);
      assertFalse(boundingBox2dA.intersects(boundingBox2dG));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructors()
   {
      Point2d lowerLeftPoint = new Point2d(0.0, 0.0);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2dA = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      // identical boxes
      double[] min = {0.0, 0.0};
      double[] max = {2.0, 2.0};
      BoundingBox2d boundingBox2dB = new BoundingBox2d(min, max);
      assertTrue(boundingBox2dA.intersects(boundingBox2dB));

      Point2d expectedCenterPoint = new Point2d();
      boundingBox2dA.getCenterPointCopy(expectedCenterPoint);
      Point2d actualCenterPoint = new Point2d();
      boundingBox2dB.getCenterPointCopy(actualCenterPoint);
      assertEquals(expectedCenterPoint.getX(), actualCenterPoint.getX(), epsilon);
      assertEquals(expectedCenterPoint.getY(), actualCenterPoint.getY(), epsilon);

      // identical boxes
      BoundingBox2d boundingBox2dC = new BoundingBox2d(boundingBox2dA);
      assertTrue(boundingBox2dA.intersects(boundingBox2dC));

      expectedCenterPoint = new Point2d();
      boundingBox2dA.getCenterPointCopy(expectedCenterPoint);
      actualCenterPoint = new Point2d();
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
      Point2d lowerLeftPoint = new Point2d(random.nextDouble(), random.nextDouble());
      Point2d upperRightPoint = new Point2d(lowerLeftPoint);
      upperRightPoint.add(new Vector2d(random.nextDouble(), random.nextDouble()));
      BoundingBox2d boundingBox2d = new BoundingBox2d(lowerLeftPoint, upperRightPoint);

      Point2d point = new Point2d();

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

      Point2d average = new Point2d();
      average.interpolate(lowerLeftPoint, upperRightPoint, 0.5);
      boundingBox2d.getPointGivenParameters(point, 0.5, 0.5);
      assertTrue(average.epsilonEquals(point, 0.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSet()
   {
      Point2d lowerLeftPoint = new Point2d(0.0123, 0.0456);
      Point2d upperRightPoint = new Point2d(2.0, 2.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d();
      boundingBox2d.set(lowerLeftPoint.getX(), lowerLeftPoint.getY(), upperRightPoint.getX(), upperRightPoint.getY());

      BoundingBox2d boundingBoxCopy = new BoundingBox2d();
      boundingBoxCopy.set(boundingBox2d);

      Point2d minPoint = new Point2d();
      boundingBoxCopy.getMinPoint(minPoint);
      assertEquals(minPoint, lowerLeftPoint);

      Point2d maxPoint = new Point2d();
      boundingBoxCopy.getMaxPoint(maxPoint);
      assertEquals(maxPoint, upperRightPoint);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetDiagonalDistanceSquared()
   {
      Point2d lowerLeftPoint = new Point2d(1.0, 7.0);
      Point2d upperRightPoint = new Point2d(4.0, 11.0);
      BoundingBox2d boundingBox2d = new BoundingBox2d();
      boundingBox2d.set(lowerLeftPoint.getX(), lowerLeftPoint.getY(), upperRightPoint.getX(), upperRightPoint.getY());

      assertEquals(25.0, boundingBox2d.getDiagonalLengthSquared(), 1e-7);
   }

   public static void main(String[] args)
   {
      String targetTests = "us.ihmc.robotics.geometry.BoundingBox2dTest";
      String targetClasses = "us.ihmc.robotics.geometry.BoundingBox2d";
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }
}
