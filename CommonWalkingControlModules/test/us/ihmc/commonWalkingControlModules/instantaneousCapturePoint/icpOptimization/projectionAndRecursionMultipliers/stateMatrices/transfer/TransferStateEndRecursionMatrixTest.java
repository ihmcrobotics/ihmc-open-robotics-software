package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferStateEndRecursionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;

import java.util.ArrayList;
import java.util.Random;

public class TransferStateEndRecursionMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      TransferStateEndRecursionMatrix transferStateEndRecursionMatrix = new TransferStateEndRecursionMatrix();

      Assert.assertEquals("", 4, transferStateEndRecursionMatrix.numRows);
      Assert.assertEquals("", 1, transferStateEndRecursionMatrix.numCols);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      DenseMatrix64F shouldBe = new DenseMatrix64F(4, 1);
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      Random random = new Random();
      int iters = 100;
      double omega0 = 3.0;


      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuraiton", registry));

      TransferStateEndRecursionMatrix transferStateEndRecursionMatrix = new TransferStateEndRecursionMatrix();

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.5 * random.nextDouble();

         doubleSupportSplitRatio.set(splitRatio);

         double currentDoubleSupportDuration = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(currentDoubleSupportDuration);

         String name = "splitRatio = " + splitRatio + ",\n doubleSupportDuration = " + currentDoubleSupportDuration;

         boolean useInitialICP = false;

         transferStateEndRecursionMatrix.compute(currentDoubleSupportDuration, omega0, useInitialICP);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(-omega0 * currentDoubleSupportDuration));
         shouldBe.set(1, 0, omega0 * Math.exp(-omega0 * currentDoubleSupportDuration));
         shouldBe.set(2, 0, 1.0);
         shouldBe.set(3, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferStateEndRecursionMatrix, epsilon);

         transferStateEndRecursionMatrix.reset();
         transferStateEndRecursionMatrix.compute(doubleSupportDurations, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferStateEndRecursionMatrix, epsilon);

         useInitialICP = true;

         transferStateEndRecursionMatrix.compute(currentDoubleSupportDuration, omega0, useInitialICP);
         shouldBe.zero();
         shouldBe.set(2, 0, 1.0);
         shouldBe.set(3, 0, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferStateEndRecursionMatrix, epsilon);

         transferStateEndRecursionMatrix.reset();
         transferStateEndRecursionMatrix.compute(doubleSupportDurations, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferStateEndRecursionMatrix, epsilon);

         transferStateEndRecursionMatrix.reset();
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, transferStateEndRecursionMatrix, epsilon);
      }
   }

   public static void main(String[] args)
   {
      MutationTestingTools.doPITMutationTestAndOpenResult(TransferStateEndRecursionMatrixTest.class);
   }
}
