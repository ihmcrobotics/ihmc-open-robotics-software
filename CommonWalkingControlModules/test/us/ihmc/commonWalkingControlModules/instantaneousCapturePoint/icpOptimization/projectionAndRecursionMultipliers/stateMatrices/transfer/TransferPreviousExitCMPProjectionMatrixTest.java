package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferPreviousExitCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationTest;

import java.util.ArrayList;
import java.util.Random;

public class TransferPreviousExitCMPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      TransferPreviousExitCMPProjectionMatrix entryCMPProjectionMatrix = new TransferPreviousExitCMPProjectionMatrix(doubleSupportSplitRatio);

      Assert.assertEquals("", 4, entryCMPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, entryCMPProjectionMatrix.numCols);
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
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration", registry));

      TransferPreviousExitCMPProjectionMatrix entryCMPProjectionMatrix = new TransferPreviousExitCMPProjectionMatrix(doubleSupportSplitRatio);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.5 * random.nextDouble();

         doubleSupportSplitRatio.set(splitRatio);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);

         String name = "splitRatio = " + splitRatio + ", doubleSupportDuration = " + doubleSupportDuration;

         double initialDoubleSupport = splitRatio * doubleSupportDuration;

         boolean useInitialICP = false;

         entryCMPProjectionMatrix.compute(doubleSupportDuration, omega0, useInitialICP);
         shouldBe.zero();
         shouldBe.set(0, 0, 1.0 - Math.exp(-omega0 * initialDoubleSupport));
         shouldBe.set(1, 0, -omega0 * Math.exp(-omega0 * initialDoubleSupport));

         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);

         entryCMPProjectionMatrix.reset();
         entryCMPProjectionMatrix.compute(doubleSupportDurations, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);

         useInitialICP = true;

         entryCMPProjectionMatrix.compute(doubleSupportDuration, omega0, useInitialICP);
         shouldBe.zero();
         shouldBe.set(1, 0, -omega0);

         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);

         entryCMPProjectionMatrix.reset();
         entryCMPProjectionMatrix.compute(doubleSupportDurations, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);

         entryCMPProjectionMatrix.reset();
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, entryCMPProjectionMatrix, epsilon);
      }
   }

   public static void main(String[] args)
   {
      MutationTestingTools.doPITMutationTestAndOpenResult(TransferPreviousExitCMPProjectionMatrixTest.class);
   }
}
