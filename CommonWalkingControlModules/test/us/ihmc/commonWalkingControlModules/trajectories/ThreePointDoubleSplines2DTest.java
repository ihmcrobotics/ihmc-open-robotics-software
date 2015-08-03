package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.utilities.MemoryTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;

public class ThreePointDoubleSplines2DTest
{
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testSimpleFlatExample()
   {
      ThreePointDoubleSplines2D spline = new ThreePointDoubleSplines2D();
      
      double zHeight = 0.23;
      
      Point3d pointOne = new Point3d(0.0, 0.0, zHeight);
      Point3d pointTwo = new Point3d(1.0, 0.0, zHeight);
      
      spline.setPoints(pointOne, pointTwo);
      
      Point2d queryPoint = new Point2d(0.37, 0.0);
      double[] zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);

      assertFlatAndAtHeight(zHeight, zSlopeAndSecondDerivative);
   }

	@EstimatedDuration(duration = 0.3)
	@Test(timeout = 30000)
   public void testInitialInALineExample()
   {
      ThreePointDoubleSplines2D spline = new ThreePointDoubleSplines2D();
      
      double zHeight = 0.23;
      double slope = 1.1;
      
      Point3d pointOne = new Point3d(0.0, 0.0, zHeight);
      Point3d pointTwo = new Point3d(1.0, 0.0, zHeight + slope);
      
      spline.setPoints(pointOne, pointTwo);
      
      Point2d queryPoint = new Point2d(0.5, 0.0);
      
      double[] zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      
      assertEquals(zHeight + slope/2.0, zSlopeAndSecondDerivative[0], 1e-7);
      assertEquals(2.0 * slope, zSlopeAndSecondDerivative[1], 1e-7);
      assertEquals(0.0, zSlopeAndSecondDerivative[2], 1e-7);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testInitialInALineExampleTwo()
   {
      ThreePointDoubleSplines2D spline = new ThreePointDoubleSplines2D();
      
      double zHeight = 0.29;
      double slope = 1.6;
      
      Point3d pointOne = new Point3d(0.0, 0.0, zHeight);
      Point3d pointTwo = new Point3d(3.0, 4.0, zHeight + 5.0 * slope);
      
      spline.setPoints(pointOne, pointTwo);
      
      
      Point2d queryPoint = new Point2d(0.0, 0.0);
      
      double[] zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      
      assertFlatAndAtHeight(zHeight, zSlopeAndSecondDerivative);
      
      
      queryPoint = new Point2d(3.0, 4.0);
      
      zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      
      assertEquals(zHeight + 5.0 * slope, zSlopeAndSecondDerivative[0], 1e-7);
      assertEquals(0.0, zSlopeAndSecondDerivative[1], 1e-7);
      assertEquals(0.0, zSlopeAndSecondDerivative[2], 1e-7);
      
      
      queryPoint = new Point2d(1.5, 2.0);
      
      zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      
      assertEquals(zHeight + 2.5 * slope, zSlopeAndSecondDerivative[0], 1e-7);
      assertEquals(2.0 * slope, zSlopeAndSecondDerivative[1], 1e-7);
      assertEquals(0.0, zSlopeAndSecondDerivative[2], 1e-7);
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testInitialInALineExampleThree()
   {
      Random random = new Random(1776L);
      
      int numberOfTests = 1;
      
      for (int i=0; i<numberOfTests; i++)
      {
         Point3d pointOne = RandomTools.generateRandomPoint(random, 2.0, 2.0, 2.0);
         Point3d pointTwo = RandomTools.generateRandomPoint(random, 2.0, 2.0, 2.0);
         
         ThreePointDoubleSplines2D spline = new ThreePointDoubleSplines2D();
         spline.setPoints(pointOne, pointTwo);
         
         for (double alpha = 0.0; alpha<=1.0; alpha=alpha+0.01)
         {
            Point3d queryPoint = morph(pointOne, pointTwo, alpha);
            Point2d queryPoint2d = new Point2d(queryPoint.getX(), queryPoint.getY());
            
            double[] zSlopeAndSecondDerivativeAtQuery = spline.getZSlopeAndSecondDerivative(queryPoint2d);
            spline.setPoints(pointOne, pointTwo);            
            
            Point2d pointOne2d = projectToGround(pointOne);
            Point2d pointTwo2d = projectToGround(pointTwo);
            
            double[] zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(pointOne2d);

            assertEquals(pointOne.getZ(), zSlopeAndSecondDerivative[0], 1e-7);
            assertEquals(0.0, zSlopeAndSecondDerivative[1], 1e-7);
            assertEquals(0.0, zSlopeAndSecondDerivative[2], 1e-7);
            
            zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(pointTwo2d);

            assertEquals(pointTwo.getZ(), zSlopeAndSecondDerivative[0], 1e-7);
            assertEquals(0.0, zSlopeAndSecondDerivative[1], 1e-7);
            assertEquals(0.0, zSlopeAndSecondDerivative[2], 1e-7);
            
            zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint2d);

            assertEquals(zSlopeAndSecondDerivativeAtQuery[0], zSlopeAndSecondDerivative[0], 1e-7);
            assertEquals(zSlopeAndSecondDerivativeAtQuery[1], zSlopeAndSecondDerivative[1], 1e-7);
            assertEquals(zSlopeAndSecondDerivativeAtQuery[2], zSlopeAndSecondDerivative[2], 1e-7);
         }
      }
   }

	@EstimatedDuration(duration = 0.4)
	@Test(timeout = 30000)
   public void testAFewQueriesOnFlatExampleOne()
   {
      double zHeight = 1.3;
      
      Point3d pointOne = new Point3d(0.0, 0.0, zHeight);
      Point3d pointTwo = new Point3d(1.0, 0.0, zHeight);
      
      ThreePointDoubleSplines2D spline = new ThreePointDoubleSplines2D();
      spline.setPoints(pointOne, pointTwo);
      
      Point2d queryPoint = new Point2d(0.5, 0.0);
      double[] zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      assertFlatAndAtHeight(zHeight, zSlopeAndSecondDerivative);
      
      Point3d pointThree = new Point3d(2.0, 0.0, zHeight);
      spline.setPoints(pointTwo, pointThree);

      queryPoint = new Point2d(1.5, 0.0);
      zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      assertFlatAndAtHeight(zHeight, zSlopeAndSecondDerivative);
      
      Point3d pointFour = new Point3d(3.0, 7.0, zHeight);
      spline.setPoints(pointThree, pointFour);

      queryPoint = new Point2d(2.5, 0.1);
      zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
      assertFlatAndAtHeight(zHeight, zSlopeAndSecondDerivative);
   }
   
   
   @Ignore // Not passing!

	@EstimatedDuration
	@Test(timeout=300000)
   public void testAFewQueries()
   {
      Random random = new Random(1776L);
      
      Point3d pointOne = RandomTools.generateRandomPoint(random, 20.0, 20.0, 2.0);
      Point3d pointTwo = RandomTools.generateRandomPoint(random, 20.0, 20.0, 2.0);
      
      ThreePointDoubleSplines2D spline = new ThreePointDoubleSplines2D();
      spline.setPoints(pointOne, pointTwo);
      
      Point2d previousQueryPoint = RandomTools.generateRandomPoint2d(random, 20.0, 20.0);
      double[] previousZSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(previousQueryPoint);
      
      int numberOfPoints = 1000;
      
      for (int i=0; i<numberOfPoints; i++)
      {
         pointOne = new Point3d(pointTwo);
         pointTwo = RandomTools.generateRandomPoint(random, 20.0, 20.0, 2.0);
         spline.setPoints(pointOne, pointTwo);

         double[] zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(projectToGround(pointOne));
         assertFlatAndAtHeight(pointOne.getZ(), zSlopeAndSecondDerivative);
         
         zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(projectToGround(pointTwo));
         assertFlatAndAtHeight(pointTwo.getZ(), zSlopeAndSecondDerivative);
         
         zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(previousQueryPoint);
         checkEqual(previousZSlopeAndSecondDerivative, zSlopeAndSecondDerivative);
         
         Point2d queryPoint = RandomTools.generateRandomPoint2d(random, 20.0, 20.0);
         zSlopeAndSecondDerivative = spline.getZSlopeAndSecondDerivative(queryPoint);
         
         previousQueryPoint = queryPoint;
         previousZSlopeAndSecondDerivative = zSlopeAndSecondDerivative;
      }

   }

   private void checkEqual(double[] previousZSlopeAndSecondDerivative, double[] zSlopeAndSecondDerivative)
   {
      assertEquals(previousZSlopeAndSecondDerivative[0], zSlopeAndSecondDerivative[0], 1e-7);
      assertEquals(previousZSlopeAndSecondDerivative[1], zSlopeAndSecondDerivative[1], 1e-7);
      assertEquals(previousZSlopeAndSecondDerivative[2], zSlopeAndSecondDerivative[2], 1e-7);
      
   }

   private void assertFlatAndAtHeight(double zHeight, double[] zSlopeAndSecondDerivative)
   {
      assertEquals(zHeight, zSlopeAndSecondDerivative[0], 1e-7);
      assertEquals(0.0, zSlopeAndSecondDerivative[1], 1e-7);
      assertEquals(0.0, zSlopeAndSecondDerivative[2], 1e-7);
   }
   
   
   public static Point3d morph(Point3d point1, Point3d point2, double alpha)
   {
      Point3d pointA = new Point3d(point1);
      Point3d pointB = new Point3d(point2);

      pointA.scale(1.0 - alpha);
      pointB.scale(alpha);
      pointA.add(pointB);


      return pointA;
   }
   
   public static Point2d projectToGround(Point3d point)
   {
      return new Point2d(point.getX(), point.getY());
   }
   

}
