package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer.TransferExitCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;
import us.ihmc.tools.testing.TestPlanAnnotations.ContinuousIntegrationTest;

import java.util.ArrayList;
import java.util.Random;

public class TransferExitCMPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);

      TransferExitCMPProjectionMatrix transferExitCMPProjectionMatrix = new TransferExitCMPProjectionMatrix(doubleSupportSplitRatio);

      Assert.assertEquals("", 4, transferExitCMPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, transferExitCMPProjectionMatrix.numCols);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      DenseMatrix64F shouldBe = new DenseMatrix64F(4, 1);
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      Random random = new Random();
      int iters = 100;

      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration", registry));

      TransferExitCMPProjectionMatrix transferExitCMPProjectionMatrix = new TransferExitCMPProjectionMatrix(doubleSupportSplitRatio);

      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.5 * random.nextDouble();
         double exitRatio = 0.5 * random.nextDouble();

         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);

         String name = "splitRatio = " + splitRatio + ", exitRatio = " + exitRatio + ",\n doubleSupportDuration = " + doubleSupportDuration;
         boolean useTwoCMPs = false;
         boolean useInitialICP = false;

         double initialDoubleSupport = splitRatio * doubleSupportDuration;
         double endOfDoubleSupport = (1.0 - splitRatio) * doubleSupportDuration;

         transferExitCMPProjectionMatrix.compute(doubleSupportDuration, useTwoCMPs, omega0, useInitialICP);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(-omega0 * initialDoubleSupport) * (1.0 - Math.exp(-omega0 * endOfDoubleSupport)));
         shouldBe.set(1, 0, omega0 * Math.exp(-omega0 * initialDoubleSupport) * (1.0 - Math.exp(-omega0 * endOfDoubleSupport)));
         shouldBe.set(3, 0, -omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferExitCMPProjectionMatrix, epsilon);
         transferExitCMPProjectionMatrix.reset();
         transferExitCMPProjectionMatrix.compute(doubleSupportDurations, useTwoCMPs, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferExitCMPProjectionMatrix, epsilon);

         useTwoCMPs = true;

         transferExitCMPProjectionMatrix.compute(doubleSupportDuration, useTwoCMPs, omega0, useInitialICP);
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, transferExitCMPProjectionMatrix, epsilon);
         transferExitCMPProjectionMatrix.reset();
         transferExitCMPProjectionMatrix.compute(doubleSupportDurations, useTwoCMPs, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferExitCMPProjectionMatrix, epsilon);


         useInitialICP = true;
         useTwoCMPs = false;

         transferExitCMPProjectionMatrix.compute(doubleSupportDuration, useTwoCMPs, omega0, useInitialICP);
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, transferExitCMPProjectionMatrix, epsilon);
         transferExitCMPProjectionMatrix.reset();
         transferExitCMPProjectionMatrix.compute(doubleSupportDurations, useTwoCMPs, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferExitCMPProjectionMatrix, epsilon);

         useTwoCMPs = true;

         transferExitCMPProjectionMatrix.compute(doubleSupportDuration, useTwoCMPs, omega0, useInitialICP);
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, transferExitCMPProjectionMatrix, epsilon);
         transferExitCMPProjectionMatrix.reset();
         transferExitCMPProjectionMatrix.compute(doubleSupportDurations, useTwoCMPs, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferExitCMPProjectionMatrix, epsilon);

         transferExitCMPProjectionMatrix.reset();
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, transferExitCMPProjectionMatrix, epsilon);
      }
   }

   public static void main(String[] args)
   {
      MutationTestingTools.doPITMutationTestAndOpenResult(TransferExitCMPProjectionMatrixTest.class);
   }
}
