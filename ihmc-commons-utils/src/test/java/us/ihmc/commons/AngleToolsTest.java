package us.ihmc.commons;


import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;

import static org.junit.jupiter.api.Assertions.*;

public class AngleToolsTest
{
   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
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

   @Test
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

   @Test
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

   @Test
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

   @Test
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

   @Test
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

   @Test
   public void testGenerateArrayOfTestAngles()
   {
      double[] anglesWithoutZeroOrPlusMinusPi = AngleTools.generateArrayOfTestAngles(100, 1e-1, false, false);
      int i = 0;
      for (double angle : anglesWithoutZeroOrPlusMinusPi)
      {
         assertTrue(angle != 0.0, "array index = " + i);
         assertTrue(angle != Math.PI, "array index = " + i);
         assertTrue(angle != -Math.PI, "array index = " + i);
         assertTrue(angle != 2.0 * Math.PI, "array index = " + i);
         assertTrue(angle != -2.0 * Math.PI, "array index = " + i);
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
         assertTrue(angle != 0.0, "array index = " + i);
         assertTrue(angle != 2.0 * Math.PI, "array index = " + i);
         assertTrue(angle != -2.0 * Math.PI, "array index = " + i);
         i++;
      }

      assertEquals(1, numberOfAnglesEqualToPlusPI, "Should have found one angle equal to +PI!");
      assertEquals(1, numberOfAnglesEqualToMinusPI, "Should have found one angle equal to -PI!");

      double[] anglesWithoutPlusMinusPi = AngleTools.generateArrayOfTestAngles(100, 1e-1, true, false);
      int numberOfAnglesEqualToZero = 0;
      i = 0;

      for (double angle : anglesWithoutPlusMinusPi)
      {
         if (angle == 0.0)
            numberOfAnglesEqualToZero++;
         assertTrue(angle != Math.PI, "array index = " + i);
         assertTrue(angle != -Math.PI, "array index = " + i);
         assertTrue(angle != 2.0 * Math.PI, "array index = " + i);
         assertTrue(angle != -2.0 * Math.PI, "array index = " + i);
         i++;
      }

      assertEquals(1, numberOfAnglesEqualToZero, "Should have found one angle equal to 0.0!");


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

         assertTrue(angle != 2.0 * Math.PI, "array index = " + i);
         assertTrue(angle != -2.0 * Math.PI, "array index = " + i);
         i++;
      }

      assertEquals(1, numberOfAnglesEqualToZero, "Should have found one angle equal to 0.0!");
      assertEquals(1, numberOfAnglesEqualToPlusPI, "Should have found one angle equal to +PI!");
      assertEquals(1, numberOfAnglesEqualToMinusPI, "Should have found one angle equal to -PI!");

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

      assertEquals(1, numberOfAnglesEqualToZero, "Should have found one angle equal to 0.0!");
      assertEquals(1, numberOfAnglesEqualToPlusPI, "Should have found one angle equal to +PI!");
      assertEquals(1, numberOfAnglesEqualToMinusPI, "Should have found one angle equal to -PI!");
      assertEquals(1, numberOfAnglesEqualToPlus2PI, "Should have found one angle equal to +2PI!");
      assertEquals(1, numberOfAnglesEqualToMinus2PI, "Should have found one angle equal to -2PI!");
   }

   @Test
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

   @Test
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

   @Test
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

   @Test
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

   @Test
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

   @Test
   public void testAngleFromZeroToTwoPi()
   {
      assertEquals(0.0, AngleTools.angleFromZeroToTwoPi(0.0, 0.0), 1e-7, "not equal");
      assertEquals(Math.PI / 4.0, AngleTools.angleFromZeroToTwoPi(1.0, 1.0), 1e-7, "not equal");
      assertEquals(7.0 * Math.PI / 4.0, AngleTools.angleFromZeroToTwoPi(1.0, -1.0), 1e-7,"not equal");
   }
   
   @Test
   public void testCalculateHeading()
   {
	   FramePose2D start = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   FramePoint2D end = new FramePoint2D(ReferenceFrame.getWorldFrame(), 1.0, 1.0);
	   double heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, 45.0, 1e-7);
	   
	   start = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2D(ReferenceFrame.getWorldFrame(), 1.0, 0.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, 0.0, 1e-7);
	   
	   start = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, 0.0, 1e-7);
	   
	   start = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 1.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, 90.0, 1e-7);

	   start = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, -1.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, -90.0, 1e-7);
	   
	   start = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2D(ReferenceFrame.getWorldFrame(), -1.0, -1.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, -135.0, 1e-7);

	   start = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2D(ReferenceFrame.getWorldFrame(), -1.0, 0.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, -180.0, 1e-7);
	   
	   start = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2D(ReferenceFrame.getWorldFrame(), 1.0, -1.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, -45, 1e-7);
	   
	   start = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(0.0,0.0), 0.0);
	   end = new FramePoint2D(ReferenceFrame.getWorldFrame(), -1.0, 1.0);
	   heading = Math.toDegrees(AngleTools.calculateHeading(start, end, 0.0, 0.0));
	   assertEquals(heading, 135.0, 1e-7);
   }

}
