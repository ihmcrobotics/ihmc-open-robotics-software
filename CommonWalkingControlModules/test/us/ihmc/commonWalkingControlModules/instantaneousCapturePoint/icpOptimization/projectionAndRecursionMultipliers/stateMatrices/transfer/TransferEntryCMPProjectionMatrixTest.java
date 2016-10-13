package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.transfer;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;

public class TransferEntryCMPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);

      TransferEntryCMPProjectionMatrix transferEntryCMPProjectionMatrix = new TransferEntryCMPProjectionMatrix(doubleSupportSplitRatio);

      Assert.assertEquals("", 4, transferEntryCMPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, transferEntryCMPProjectionMatrix.numCols);
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

      TransferEntryCMPProjectionMatrix transferEntryCMPProjectionMatrix = new TransferEntryCMPProjectionMatrix(doubleSupportSplitRatio);

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

         transferEntryCMPProjectionMatrix.compute(doubleSupportDuration, useTwoCMPs, omega0, useInitialICP);
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, transferEntryCMPProjectionMatrix, epsilon);
         transferEntryCMPProjectionMatrix.reset();
         transferEntryCMPProjectionMatrix.compute(doubleSupportDurations, useTwoCMPs, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferEntryCMPProjectionMatrix, epsilon);

         useTwoCMPs = true;

         transferEntryCMPProjectionMatrix.compute(doubleSupportDuration, useTwoCMPs, omega0, useInitialICP);
         shouldBe.zero();
         shouldBe.set(0, 0, Math.exp(-omega0 * initialDoubleSupport) * (1.0 - Math.exp(-omega0 * endOfDoubleSupport)));
         shouldBe.set(1, 0, omega0 * Math.exp(-omega0 * initialDoubleSupport) * (1.0 - Math.exp(-omega0 * endOfDoubleSupport)));
         shouldBe.set(3, 0, -omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferEntryCMPProjectionMatrix, epsilon);
         transferEntryCMPProjectionMatrix.reset();
         transferEntryCMPProjectionMatrix.compute(doubleSupportDurations, useTwoCMPs, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferEntryCMPProjectionMatrix, epsilon);



         useTwoCMPs = false;
         useInitialICP = true;

         transferEntryCMPProjectionMatrix.compute(doubleSupportDuration, useTwoCMPs, omega0, useInitialICP);
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, transferEntryCMPProjectionMatrix, epsilon);
         transferEntryCMPProjectionMatrix.reset();
         transferEntryCMPProjectionMatrix.compute(doubleSupportDurations, useTwoCMPs, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferEntryCMPProjectionMatrix, epsilon);

         useTwoCMPs = true;

         transferEntryCMPProjectionMatrix.compute(doubleSupportDuration, useTwoCMPs, omega0, useInitialICP);
         shouldBe.zero();
         shouldBe.set(3, 0, -omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferEntryCMPProjectionMatrix, epsilon);
         transferEntryCMPProjectionMatrix.reset();
         transferEntryCMPProjectionMatrix.compute(doubleSupportDurations, useTwoCMPs, omega0, useInitialICP);
         JUnitTools.assertMatrixEquals(name, shouldBe, transferEntryCMPProjectionMatrix, epsilon);

         transferEntryCMPProjectionMatrix.reset();
         shouldBe.zero();
         JUnitTools.assertMatrixEquals(name, shouldBe, transferEntryCMPProjectionMatrix, epsilon);
      }
   }
   
   public static void main(String[] args)
   {
      MutationTestingTools.doPITMutationTestAndOpenResult(TransferEntryCMPProjectionMatrixTest.class);
   }
}
