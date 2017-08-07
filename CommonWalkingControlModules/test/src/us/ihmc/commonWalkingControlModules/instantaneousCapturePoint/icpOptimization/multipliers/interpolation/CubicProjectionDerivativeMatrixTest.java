package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.interpolation;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.testing.JUnitTools;

public class CubicProjectionDerivativeMatrixTest
{
   private static final double epsilon = 0.00005;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      CubicDerivativeMatrix cubicProjectionMatrix = new CubicDerivativeMatrix();

      Assert.assertEquals("", 4, cubicProjectionMatrix.numCols);
      Assert.assertEquals("", 1, cubicProjectionMatrix.numRows);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testSegmentDuration()
   {
      CubicDerivativeMatrix cubicProjectionMatrix = new CubicDerivativeMatrix();

      Random random = new Random();
      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         double duration = 10.0 * random.nextDouble();

         cubicProjectionMatrix.setSegmentDuration(duration);

         Assert.assertEquals(duration, cubicProjectionMatrix.getSegmentDuration(), epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      CubicDerivativeMatrix cubicProjectionMatrix = new CubicDerivativeMatrix();

      DenseMatrix64F cubicSplineMatrix = new DenseMatrix64F(4, 4);
      DenseMatrix64F cubicTimeMatrix = new DenseMatrix64F(1, 4);
      DenseMatrix64F shouldBe = new DenseMatrix64F(1, 4);

      Random random = new Random();
      int iters = 100;

      for (int i = 0; i < iters; i++)
      {
         double duration = 10.0 * random.nextDouble();
         double time = duration * random.nextDouble();

         cubicProjectionMatrix.setSegmentDuration(duration);
         cubicProjectionMatrix.update(time);

         cubicSplineMatrix.zero();
         cubicTimeMatrix.zero();
         shouldBe.zero();

         cubicTimeMatrix.set(0, 0, 3.0 * Math.pow(time, 2.0));
         cubicTimeMatrix.set(0, 1, 2.0 * time);
         cubicTimeMatrix.set(0, 2, 1.0);
         cubicTimeMatrix.set(0, 3, 0.0);

         cubicSplineMatrix.set(0, 0, 2.0 / Math.pow(duration, 3.0));
         cubicSplineMatrix.set(0, 1, 1.0 / Math.pow(duration, 2.0));
         cubicSplineMatrix.set(0, 2, -2.0 / Math.pow(duration, 3.0));
         cubicSplineMatrix.set(0, 3, 1.0 / Math.pow(duration, 2.0));

         cubicSplineMatrix.set(1, 0, -3.0 / Math.pow(duration, 2.0));
         cubicSplineMatrix.set(1, 1, -2.0 / Math.pow(duration, 1.0));
         cubicSplineMatrix.set(1, 2, 3.0 / Math.pow(duration, 2.0));
         cubicSplineMatrix.set(1, 3, -1.0 / Math.pow(duration, 1.0));

         cubicSplineMatrix.set(2, 0, 0.0);
         cubicSplineMatrix.set(2, 1, 1.0);
         cubicSplineMatrix.set(2, 2, 0.0);
         cubicSplineMatrix.set(2, 3, 0.0);

         cubicSplineMatrix.set(3, 0, 1.0);
         cubicSplineMatrix.set(3, 1, 0.0);
         cubicSplineMatrix.set(3, 2, 0.0);
         cubicSplineMatrix.set(3, 3, 0.0);

         CommonOps.mult(cubicTimeMatrix, cubicSplineMatrix, shouldBe);

         JUnitTools.assertMatrixEquals(shouldBe, cubicProjectionMatrix, epsilon);
      }
   }
}
