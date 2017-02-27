package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class AngleToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructor()
           throws NoSuchMethodException, SecurityException, InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      // Screw you clover, I can test the private constructor(!)
      assertEquals(1, AngleTools.class.getDeclaredConstructors().length);
      Constructor<AngleTools> constructor = AngleTools.class.getDeclaredConstructor();
      assertTrue(Modifier.isPrivate(constructor.getModifiers()));
      constructor.setAccessible(true);
      constructor.newInstance();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeAngleDifferenceMinusTwoPiToZero()
   {
      Random random = new Random(123456);
      double angleA = Math.PI;
      double angleB = Math.PI / 2;
      double expectedReturn = -2.0 * Math.PI + Math.PI / 2;
      double actualReturn = AngleTools.computeAngleDifferenceMinusTwoPiToZero(angleA, angleB);
      assertEquals(expectedReturn, actualReturn, Double.MIN_VALUE);

      for (int i = 0; i < 25; i++)
      {
         angleA = RandomNumbers.nextDouble(random, -128.0, 128.0);
         angleB = RandomNumbers.nextDouble(random, -128.0, 128.0);

         double ret = AngleTools.computeAngleDifferenceMinusTwoPiToZero(angleA, angleB);

         assertTrue(ret <= 0.0);
         assertTrue(ret >= -2.0 * Math.PI);
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeAngleDifferenceMinusPiToPi()
   {
      Random random = new Random(123456);
      double angleA = Math.PI;
      double angleB = Math.PI / 2;
      double expectedReturn = Math.PI / 2;
      double actualReturn = AngleTools.computeAngleDifferenceMinusPiToPi(angleA, angleB);
      assertEquals(expectedReturn, actualReturn, Double.MIN_VALUE);

      for (int i = 0; i < 25; i++)
      {
         angleA = RandomNumbers.nextDouble(random, -128.0, 128.0);
         angleB = RandomNumbers.nextDouble(random, -128.0, 128.0);

         double ret = AngleTools.computeAngleDifferenceMinusPiToPi(angleA, angleB);

         assertTrue(ret <= Math.PI);
         assertTrue(ret >= -Math.PI);
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeAngleDifferenceMinusPiToPiUsingTrim()
   {
      Random random = new Random(123456);
      double angleA = Math.PI;
      double angleB = Math.PI / 2;
      double expectedReturn = Math.PI / 2;
      double actualReturn = AngleTools.trimAngleMinusPiToPi(angleA - angleB);
      assertEquals(expectedReturn, actualReturn, Double.MIN_VALUE);

      for (int i = 0; i < 25; i++)
      {
         angleA = RandomNumbers.nextDouble(random, -128.0, 128.0);
         angleB = RandomNumbers.nextDouble(random, -128.0, 128.0);

         double ret = AngleTools.trimAngleMinusPiToPi(angleA - angleB);

         assertTrue(ret <= Math.PI);
         assertTrue(ret >= -Math.PI);
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFindClosestNinetyDegreeYaw()
   {
      double yawInRadians;
      int expectedReturn, actualReturn;


      yawInRadians = -0.4 * Math.PI;
      expectedReturn = 3;
      actualReturn = AngleTools.findClosestNinetyDegreeYaw(yawInRadians);
      assertEquals(expectedReturn, actualReturn);


      yawInRadians = 0.6 * Math.PI;
      expectedReturn = 1;
      actualReturn = AngleTools.findClosestNinetyDegreeYaw(yawInRadians);
      assertEquals(expectedReturn, actualReturn);

      yawInRadians = 0.9 * Math.PI;
      expectedReturn = 2;
      actualReturn = AngleTools.findClosestNinetyDegreeYaw(yawInRadians);
      assertEquals(expectedReturn, actualReturn);

      yawInRadians = 2.4 * Math.PI;
      expectedReturn = 1;
      actualReturn = AngleTools.findClosestNinetyDegreeYaw(yawInRadians);
      assertEquals(expectedReturn, actualReturn);

      yawInRadians = 4 * Math.PI;
      expectedReturn = 0;
      actualReturn = AngleTools.findClosestNinetyDegreeYaw(yawInRadians);
      assertEquals(expectedReturn, actualReturn);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateRandomAngle()
   {
      Random random = new Random(0);

      for (int i = 0; i < 25; i++)
      {
         double randomAngle = AngleTools.generateRandomAngle(random);
         assertTrue(randomAngle <= 2.0 * Math.PI);
         assertTrue(randomAngle >= -2.0 * Math.PI);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGenerateArrayOfTestAngles()
   {
      double[] anglesWithoutZeroOrPlusMinusPi = AngleTools.generateArrayOfTestAngles(100, 1e-1, false, false);
      int i = 0;
      for (double angle : anglesWithoutZeroOrPlusMinusPi)
      {
         assertTrue("array index = " + i, angle != 0.0);
         assertTrue("array index = " + i, angle != Math.PI);
         assertTrue("array index = " + i, angle != -Math.PI);
         assertTrue("array index = " + i, angle != 2.0 * Math.PI);
         assertTrue("array index = " + i, angle != -2.0 * Math.PI);
         i++;
      }

      double[] anglesWithoutZero = AngleTools.generateArrayOfTestAngles(100, 1e-1, false, true);
      int numberOfAnglesEqualToPlusPI = 0;
      int numberOfAnglesEqualToMinusPI = 0;
      i = 0;

      for (double angle : anglesWithoutZero)
      {
         if (angle == Math.PI)
            numberOfAnglesEqualToPlusPI++;
         if (angle == -Math.PI)
            numberOfAnglesEqualToMinusPI++;
         assertTrue("array index = " + i, angle != 0.0);
         assertTrue("array index = " + i, angle != 2.0 * Math.PI);
         assertTrue("array index = " + i, angle != -2.0 * Math.PI);
         i++;
      }

      assertEquals("Should have found one angle equal to +PI!", 1, numberOfAnglesEqualToPlusPI);
      assertEquals("Should have found one angle equal to -PI!", 1, numberOfAnglesEqualToMinusPI);

      double[] anglesWithoutPlusMinusPi = AngleTools.generateArrayOfTestAngles(100, 1e-1, true, false);
      int numberOfAnglesEqualToZero = 0;
      i = 0;

      for (double angle : anglesWithoutPlusMinusPi)
      {
         if (angle == 0.0)
            numberOfAnglesEqualToZero++;
         assertTrue("array index = " + i, angle != Math.PI);
         assertTrue("array index = " + i, angle != -Math.PI);
         assertTrue("array index = " + i, angle != 2.0 * Math.PI);
         assertTrue("array index = " + i, angle != -2.0 * Math.PI);
         i++;
      }

      assertEquals("Should have found one angle equal to 0.0!", 1, numberOfAnglesEqualToZero);


      double[] allAnglesExceptPlusMinus2PI = AngleTools.generateArrayOfTestAngles(100, 1e-1, true, true);
      i = 0;
      numberOfAnglesEqualToZero = 0;
      numberOfAnglesEqualToPlusPI = 0;
      numberOfAnglesEqualToMinusPI = 0;

      for (double angle : allAnglesExceptPlusMinus2PI)
      {
         if (angle == 0.0)
            numberOfAnglesEqualToZero++;
         if (angle == Math.PI)
            numberOfAnglesEqualToPlusPI++;
         if (angle == -Math.PI)
            numberOfAnglesEqualToMinusPI++;

         assertTrue("array index = " + i, angle != 2.0 * Math.PI);
         assertTrue("array index = " + i, angle != -2.0 * Math.PI);
         i++;
      }

      assertEquals("Should have found one angle equal to 0.0!", 1, numberOfAnglesEqualToZero);
      assertEquals("Should have found one angle equal to +PI!", 1, numberOfAnglesEqualToPlusPI);
      assertEquals("Should have found one angle equal to -PI!", 1, numberOfAnglesEqualToMinusPI);

      double[] allAnglesIncludingPlusMinus2PI = AngleTools.generateArrayOfTestAngles(100, 0.0, true, true);
      i = 0;
      numberOfAnglesEqualToZero = 0;
      numberOfAnglesEqualToPlusPI = 0;
      numberOfAnglesEqualToMinusPI = 0;
      int numberOfAnglesEqualToPlus2PI = 0;
      int numberOfAnglesEqualToMinus2PI = 0;

      for (double angle : allAnglesIncludingPlusMinus2PI)
      {
         if (angle == 0.0)
            numberOfAnglesEqualToZero++;
         if (angle == Math.PI)
            numberOfAnglesEqualToPlusPI++;
         if (angle == -Math.PI)
            numberOfAnglesEqualToMinusPI++;
         if (angle == 2.0 * Math.PI)
            numberOfAnglesEqualToPlus2PI++;
         if (angle == -2.0 * Math.PI)
            numberOfAnglesEqualToMinus2PI++;

         i++;
      }

      assertEquals("Should have found one angle equal to 0.0!", 1, numberOfAnglesEqualToZero);
      assertEquals("Should have found one angle equal to +PI!", 1, numberOfAnglesEqualToPlusPI);
      assertEquals("Should have found one angle equal to -PI!", 1, numberOfAnglesEqualToMinusPI);
      assertEquals("Should have found one angle equal to +2PI!", 1, numberOfAnglesEqualToPlus2PI);
      assertEquals("Should have found one angle equal to -2PI!", 1, numberOfAnglesEqualToMinus2PI);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testShiftAngleToStartOfRange()
   {
      double angleToShift = 0.5;
      double startOfAngleRange = Math.PI;
      double expectedReturn = 3.0 * Math.PI - (Math.PI - 0.5);
      double actualReturn = AngleTools.shiftAngleToStartOfRange(angleToShift, startOfAngleRange);
      assertEquals(expectedReturn, actualReturn, 1e-12);


      angleToShift = 12.0;
      startOfAngleRange = 0.5 * Math.PI;
      expectedReturn = 12.0 - 2 * Math.PI;
      actualReturn = AngleTools.shiftAngleToStartOfRange(angleToShift, startOfAngleRange);
      assertEquals(expectedReturn, actualReturn, 1e-12);

      angleToShift = 0.8;
      startOfAngleRange = -0.25 * Math.PI;
      expectedReturn = 0.8;
      actualReturn = AngleTools.shiftAngleToStartOfRange(angleToShift, startOfAngleRange);
      assertEquals(expectedReturn, actualReturn, 1e-12);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testShiftAngleToStartOfRangeUnitless()
   {
      double range = Math.pow(2.0, 13.0);

      double endOfAngleRange = range;
      double angleToShift = 1.6 * range;
      double expectedReturn = 0.6 * range;
      double actualReturn = AngleTools.shiftAngleToStartOfRange(angleToShift, 0.0, endOfAngleRange);
      assertEquals(expectedReturn, actualReturn, 1e-12);

      angleToShift = -0.4 * range;
      expectedReturn = 0.6 * range;
      actualReturn = AngleTools.shiftAngleToStartOfRange(angleToShift, 0.0, endOfAngleRange);
      assertEquals(expectedReturn, actualReturn, 1e-12);

      angleToShift = 0.4 * range;
      expectedReturn = 0.4 * range;
      actualReturn = AngleTools.shiftAngleToStartOfRange(angleToShift, 0.0, endOfAngleRange);
      assertEquals(expectedReturn, actualReturn, 1e-12);

      int iters = 1000;
      Random random = new Random();
      for (int i = 0; i < iters; i++)
      {
         double ratio = -6.0 + 12.0 * random.nextDouble();

         angleToShift = ratio * range;
         expectedReturn = angleToShift;

         if (angleToShift < 0.0)
            expectedReturn = angleToShift + Math.ceil((-angleToShift) / endOfAngleRange) * endOfAngleRange;

         if (angleToShift >= endOfAngleRange)
            expectedReturn = angleToShift - Math.floor((angleToShift) / endOfAngleRange) * endOfAngleRange;

         actualReturn = AngleTools.shiftAngleToStartOfRange(angleToShift, 0.0, endOfAngleRange);
         assertEquals(expectedReturn, actualReturn, 1e-12);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrimAngleMinusPiToPi()
   {
      Random random = new Random(0);

      for (int i = 0; i < 25; i++)
      {
         double randomAngle = RandomNumbers.nextDouble(random, -128.0, 128.0);
         randomAngle = AngleTools.trimAngleMinusPiToPi(randomAngle);
         assertTrue(randomAngle <= Math.PI);
         assertTrue(randomAngle >= -Math.PI);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeAngleAverage()
   {
      double angleA, angleB;
      double expected, actual;


      angleA = Math.PI;
      angleB = 2 * Math.PI + 0.3;
      expected = (Math.PI + 0.3) / 2.0;
      actual = AngleTools.computeAngleAverage(angleA, angleB);
      assertEquals(expected, actual, 1e-12);

      angleB = Math.PI;
      angleA = 2 * Math.PI + 0.3;
      expected = (Math.PI + 0.3) / 2.0;
      actual = AngleTools.computeAngleAverage(angleA, angleB);
      assertEquals(expected, actual, 1e-12);


      angleA = 0.3;
      angleB = Math.PI;
      expected = (Math.PI + 0.3) / 2.0;
      actual = AngleTools.computeAngleAverage(angleA, angleB);
      assertEquals(expected, actual, 1e-12);


      angleA = Math.PI - 0.3;
      angleB = -Math.PI + 0.1;
      expected = Math.PI - 0.1;
      actual = AngleTools.computeAngleAverage(angleA, angleB);
      assertEquals(expected, actual, 1e-12);

      angleB = Math.PI - 0.3;
      angleA = -Math.PI + 0.1;
      expected = Math.PI - 0.1;
      actual = AngleTools.computeAngleAverage(angleA, angleB);
      assertEquals(expected, actual, 1e-12);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAngleMinusPiToPi()
   {
      Vector2D vectorA, vectorB;
      double expected, actual;


      vectorA = new Vector2D(0.0, 1.0);
      vectorB = new Vector2D(1.0, 0.0);
      expected = -0.5 * Math.PI;
      actual = AngleTools.angleMinusPiToPi(vectorA, vectorB);
      assertEquals(expected, actual, 1e-12);

      vectorA = new Vector2D(0.0, 1.0);
      vectorB = new Vector2D(-1.0, 0.0);
      expected = 0.5 * Math.PI;
      actual = AngleTools.angleMinusPiToPi(vectorA, vectorB);
      assertEquals(expected, actual, 1e-12);


      vectorA = new Vector2D(1.0, 1.0);
      vectorB = new Vector2D(-1.0, 0.0);
      expected = 0.75 * Math.PI;
      actual = AngleTools.angleMinusPiToPi(vectorA, vectorB);
      assertEquals(expected, actual, 1e-12);



      vectorA = new Vector2D(0.0, 1.0);
      vectorB = new Vector2D(0.0, 0.0);
      expected = AngleTools.angleMinusPiToPi(vectorA, vectorB);
      assertTrue(Double.isNaN(expected));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCalculateHeading()
   {
	   FramePose2d start = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   FramePoint2d end = new FramePoint2d(ReferenceFrame.getWorldFrame(), 1.0, 1.0);
	   double heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, 45.0, 1e-7);
	   
	   start = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2d(ReferenceFrame.getWorldFrame(), 1.0, 0.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, 0.0, 1e-7);
	   
	   start = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, 0.0, 1e-7);
	   
	   start = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, 1.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, 90.0, 1e-7);

	   start = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2d(ReferenceFrame.getWorldFrame(), 0.0, -1.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, -90.0, 1e-7);
	   
	   start = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2d(ReferenceFrame.getWorldFrame(), -1.0, -1.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, -135.0, 1e-7);

	   start = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2d(ReferenceFrame.getWorldFrame(), -1.0, 0.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, -180.0, 1e-7);
	   
	   start = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2d(ReferenceFrame.getWorldFrame(), 1.0, -1.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, -45, 1e-7);
	   
	   start = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2d(ReferenceFrame.getWorldFrame(), -1.0, 1.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, 135.0, 1e-7);
   }

}
