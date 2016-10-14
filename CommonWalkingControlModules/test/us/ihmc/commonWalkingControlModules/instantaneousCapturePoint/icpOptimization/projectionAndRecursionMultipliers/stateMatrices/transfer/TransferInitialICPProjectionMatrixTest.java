package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;

import java.util.ArrayList;
import java.util.Random;

public class TransferInitialICPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      TransferInitialICPProjectionMatrix transferInitialICPProjectionMatrix = new TransferInitialICPProjectionMatrix();

      Assert.assertEquals("", 4, transferInitialICPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, transferInitialICPProjectionMatrix.numCols);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      DenseMatrix64F shouldBe = new DenseMatrix64F(4, 1);

      int iters = 100;


      TransferInitialICPProjectionMatrix transferInitialICPProjectionMatrix = new TransferInitialICPProjectionMatrix();

      double omega0 = 3.0;
      for (int i = 0; i < iters; i++)
      {
         String name = "";

         transferInitialICPProjectionMatrix.compute(omega0);
         shouldBe.zero();
         shouldBe.set(0, 0, 1.0);
         shouldBe.set(1, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferInitialICPProjectionMatrix, epsilon);

         transferInitialICPProjectionMatrix.reset();
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, transferInitialICPProjectionMatrix, epsilon);
      }
   }

   public static void main(String[] args)
   {
      MutationTestingTools.doPITMutationTestAndOpenResult(TransferInitialICPProjectionMatrixTest.class);
   }
}
