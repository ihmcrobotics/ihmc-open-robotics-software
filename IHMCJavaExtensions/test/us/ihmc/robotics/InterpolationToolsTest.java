package us.ihmc.robotics;

import static org.junit.Assert.assertEquals;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

import java.util.Random;

public class InterpolationToolsTest
{
   private static final double epsilon = 1e-7;
   private static final int iters = 1000;
   private Random random;

   @Before
   public void setUp() throws Exception
   {
      random = new Random(100L);
   }

   @After
   public void tearDown() throws Exception
   {
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPiecewiseInterpolation()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 1.0);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 0.2);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 0.8);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 0.5);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 1.1);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, 30.0);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, -0.1);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolate(boundA, boundB, -30.0);
         assertEquals(value, boundA, epsilon);

         double alpha = random.nextDouble();
         if (alpha < 0.5)
         {
            value = InterpolationTools.piecewiseInterpolate(boundA, boundB, alpha);
            assertEquals(value, boundA, epsilon);
         }
         else
         {
            value = InterpolationTools.piecewiseInterpolate(boundA, boundB, alpha);
            assertEquals(value, boundB, epsilon);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPiecewiseInterpolationRoundDown()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 1.0);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 0.2);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 0.8);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 0.5);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 1.1);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, 30.0);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, -0.1);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, -30.0);
         assertEquals(value, boundA, epsilon);

         double alpha = random.nextDouble();
         if (alpha > 0.5)
         {
            value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, alpha);
            assertEquals(value, boundB, epsilon);
         }
         else
         {
            value = InterpolationTools.piecewiseInterpolateRoundDown(boundA, boundB, alpha);
            assertEquals(value, boundA, epsilon);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearInterpolateBounds()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double value = InterpolationTools.linearInterpolate(boundA, boundB, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.linearInterpolate(boundA, boundB, 1.0);
         assertEquals(value, boundB, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLinearInterpolate()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double average = 0.5 * (boundA + boundB);

         double value = InterpolationTools.linearInterpolate(boundA, boundB, 0.5);
         assertEquals(value, average, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testHermite01InterpolateBounds()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double value = InterpolationTools.hermite01Interpolate(boundA, boundB, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.hermite01Interpolate(boundA, boundB, 1.0);
         assertEquals(value, boundB, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testHermite01Interpolate()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double average = 0.5 * (boundA + boundB);

         double value = InterpolationTools.hermite01Interpolate(boundA, boundB, 0.5);
         assertEquals(value, average, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testHermiteInterpolateBounds()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();
         double boundATangent = 100.0 * random.nextDouble();
         double boundBTangent = 100.0 * random.nextDouble();

         double value = InterpolationTools.hermiteInterpolate(boundA, boundB, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.hermiteInterpolate(boundA, boundATangent, boundB, boundBTangent, 0.0);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.hermiteInterpolate(boundA, boundB, 1.0);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.hermiteInterpolate(boundA, boundATangent, boundB, boundBTangent, 1.0);
         assertEquals(value, boundB, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testHermiteInterpolate()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double average = 0.5 * (boundA + boundB);

         double value = InterpolationTools.hermiteInterpolate(boundA, boundB, 0.5);
         assertEquals(value, average, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLogisticInterpolateBounds()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double value = InterpolationTools.logisticInterpolate(boundA, boundB, 0.0);
         assertEquals(value, boundA, 1e-1);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, 1.0);
         assertEquals(value, boundB, 1e-1);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, -0.1);
         assertEquals(value, boundA, 1e-1);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, -1.0);
         assertEquals(value, boundA, 1e-1);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, 1.1);
         assertEquals(value, boundB, 1e-1);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, 2.0);
         assertEquals(value, boundB, 1e-1);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLogisticInterpolate()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double average = 0.5 * (boundA + boundB);

         double value = InterpolationTools.logisticInterpolate(boundA, boundB, 0.5);
         assertEquals(value, average, epsilon);
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLogisticInterpolateSlopedBounds()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double slope = 10.0;

         double value = InterpolationTools.logisticInterpolate(boundA, boundB, 0.0, slope);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, 1.0, slope);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, -0.1, slope);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, -1.0, slope);
         assertEquals(value, boundA, epsilon);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, 1.1, slope);
         assertEquals(value, boundB, epsilon);

         value = InterpolationTools.logisticInterpolate(boundA, boundB, 2.0, slope);
         assertEquals(value, boundB, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLogisticInterpolateSloped()
   {
      for (int i = 0; i < iters; i++)
      {
         double boundA = 10.0 * random.nextDouble();
         double boundB = 10.0 * random.nextDouble();

         double slope = 10.0;

         double average = 0.5 * (boundA + boundB);

         double value = InterpolationTools.logisticInterpolate(boundA, boundB, 0.5, slope);
         assertEquals(value, average, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testHermiteCoefficients()
   {
      // left bound
      double h00 = InterpolationTools.hermite00Coefficient(0.0);
      assertEquals(h00, 1.0, epsilon);

      double h10 = InterpolationTools.hermite10Coefficient(0.0);
      assertEquals(h10, 0.0, epsilon);

      double h01 = InterpolationTools.hermite01Coefficient(0.0);
      assertEquals(h01, 0.0, epsilon);

      double h11 = InterpolationTools.hermite11Coefficient(0.0);
      assertEquals(h11, 0.0, epsilon);

      // right bound
      h00 = InterpolationTools.hermite00Coefficient(1.0);
      assertEquals(h00, 0.0, epsilon);

      h10 = InterpolationTools.hermite10Coefficient(1.0);
      assertEquals(h10, 0.0, epsilon);

      h01 = InterpolationTools.hermite01Coefficient(1.0);
      assertEquals(h01, 1.0, epsilon);

      h11 = InterpolationTools.hermite11Coefficient(1.0);
      assertEquals(h11, 0.0, epsilon);

      // midpoint
      h00 = InterpolationTools.hermite00Coefficient(0.5);
      assertEquals(h00, 0.5, epsilon);

      h10 = InterpolationTools.hermite10Coefficient(0.5);
      assertEquals(h10, 0.125, epsilon);

      h01 = InterpolationTools.hermite01Coefficient(0.5);
      assertEquals(h01, 0.5, epsilon);

      h11 = InterpolationTools.hermite11Coefficient(0.5);
      assertEquals(h11, -0.125, epsilon);

      for (int i = 0; i < iters; i++)
      {
         double alpha = random.nextDouble();
         h00 = InterpolationTools.hermite00Coefficient(alpha);
         double h00ShouldBe = 2.0 * Math.pow(alpha, 3.0) - 3.0 * Math.pow(alpha, 2.0) + 1.0;
         assertEquals(h00, h00ShouldBe, epsilon);

         h10 = InterpolationTools.hermite10Coefficient(alpha);
         double h10ShouldBe = Math.pow(alpha, 3.0) - 2.0 * Math.pow(alpha, 2.0) + alpha;
         assertEquals(h10, h10ShouldBe, epsilon);

         h01 = InterpolationTools.hermite01Coefficient(alpha);
         double h01ShouldBe = -2.0 * Math.pow(alpha, 3.0) + 3.0 * Math.pow(alpha, 2.0);
         assertEquals(h01, h01ShouldBe, epsilon);

         h11 = InterpolationTools.hermite11Coefficient(alpha);
         double h11ShouldBe = Math.pow(alpha, 3.0) - Math.pow(alpha, 2.0);
         assertEquals(h11, h11ShouldBe, epsilon);
      }
   }
}
