package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation;

import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class CubicSplineCoefficientMatrixTest
{
   private static final double epsilon = 0.00005;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      CubicSplineCoefficientMatrix cubicSplineMatrix = new CubicSplineCoefficientMatrix();

      Assert.assertEquals("", 4, cubicSplineMatrix.numCols);
      Assert.assertEquals("", 4, cubicSplineMatrix.numRows);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testDurationSetting()
   {
      CubicSplineCoefficientMatrix cubicSplineMatrix = new CubicSplineCoefficientMatrix();

      Random random = new Random();
      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         double duration = 10.0 * random.nextDouble();
         cubicSplineMatrix.setSegmentDuration(duration);

         Assert.assertEquals("", 2.0 / Math.pow(duration, 3.0), cubicSplineMatrix.get(0, 0), epsilon);
         Assert.assertEquals("", 1.0 / Math.pow(duration, 2.0), cubicSplineMatrix.get(0, 1), epsilon);
         Assert.assertEquals("", -2.0 / Math.pow(duration, 3.0), cubicSplineMatrix.get(0, 2), epsilon);
         Assert.assertEquals("", 1.0 / Math.pow(duration, 2.0), cubicSplineMatrix.get(0, 3), epsilon);

         Assert.assertEquals("", -3.0 / Math.pow(duration, 2.0), cubicSplineMatrix.get(1, 0), epsilon);
         Assert.assertEquals("", -2.0 / Math.pow(duration, 1.0), cubicSplineMatrix.get(1, 1), epsilon);
         Assert.assertEquals("", 3.0 / Math.pow(duration, 2.0), cubicSplineMatrix.get(1, 2), epsilon);
         Assert.assertEquals("", -1.0 / Math.pow(duration, 1.0), cubicSplineMatrix.get(1, 3), epsilon);

         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(2, 0), epsilon);
         Assert.assertEquals("", 1.0, cubicSplineMatrix.get(2, 1), epsilon);
         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(2, 2), epsilon);
         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(2, 3), epsilon);

         Assert.assertEquals("", 1.0, cubicSplineMatrix.get(3, 0), epsilon);
         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(3, 1), epsilon);
         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(3, 2), epsilon);
         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(3, 3), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testDurationCreation()
   {
      Random random = new Random();
      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         double duration = 10.0 * random.nextDouble();
         CubicSplineCoefficientMatrix cubicSplineMatrix = new CubicSplineCoefficientMatrix(duration);

         Assert.assertEquals("", 2.0 / Math.pow(duration, 3.0), cubicSplineMatrix.get(0, 0), epsilon);
         Assert.assertEquals("", 1.0 / Math.pow(duration, 2.0), cubicSplineMatrix.get(0, 1), epsilon);
         Assert.assertEquals("", -2.0 / Math.pow(duration, 3.0), cubicSplineMatrix.get(0, 2), epsilon);
         Assert.assertEquals("", 1.0 / Math.pow(duration, 2.0), cubicSplineMatrix.get(0, 3), epsilon);

         Assert.assertEquals("", -3.0 / Math.pow(duration, 2.0), cubicSplineMatrix.get(1, 0), epsilon);
         Assert.assertEquals("", -2.0 / Math.pow(duration, 1.0), cubicSplineMatrix.get(1, 1), epsilon);
         Assert.assertEquals("", 3.0 / Math.pow(duration, 2.0), cubicSplineMatrix.get(1, 2), epsilon);
         Assert.assertEquals("", -1.0 / Math.pow(duration, 1.0), cubicSplineMatrix.get(1, 3), epsilon);

         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(2, 0), epsilon);
         Assert.assertEquals("", 1.0, cubicSplineMatrix.get(2, 1), epsilon);
         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(2, 2), epsilon);
         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(2, 3), epsilon);

         Assert.assertEquals("", 1.0, cubicSplineMatrix.get(3, 0), epsilon);
         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(3, 1), epsilon);
         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(3, 2), epsilon);
         Assert.assertEquals("", 0.0, cubicSplineMatrix.get(3, 3), epsilon);
      }
   }
}
