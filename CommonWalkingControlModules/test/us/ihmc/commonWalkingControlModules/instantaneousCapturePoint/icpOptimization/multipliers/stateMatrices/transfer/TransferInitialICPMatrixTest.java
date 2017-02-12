package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

public class TransferInitialICPMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      TransferInitialICPMatrix initialICPMatrix = new TransferInitialICPMatrix();

      Assert.assertEquals("", 4, initialICPMatrix.numRows);
      Assert.assertEquals("", 1, initialICPMatrix.numCols);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      DenseMatrix64F shouldBe = new DenseMatrix64F(4, 1);
      int iters = 100;

      TransferInitialICPMatrix initialICPMatrix = new TransferInitialICPMatrix();

      for (int i = 0; i < iters; i++)
      {
         initialICPMatrix.compute();

         shouldBe.zero();
         shouldBe.set(0, 0, 1.0);
         shouldBe.set(1, 0, 0.0);

         JUnitTools.assertMatrixEquals("", shouldBe, initialICPMatrix, epsilon);

         shouldBe.zero();
         initialICPMatrix.reset();
         JUnitTools.assertMatrixEquals("", shouldBe, initialICPMatrix, epsilon);
      }
   }
}
