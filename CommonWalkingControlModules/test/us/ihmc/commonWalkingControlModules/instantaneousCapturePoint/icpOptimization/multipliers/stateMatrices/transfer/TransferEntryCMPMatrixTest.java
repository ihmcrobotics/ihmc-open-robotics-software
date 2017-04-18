package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.testing.JUnitTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class TransferEntryCMPMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("swingSplitFraction1", registry);
      swingSplitFractions.add(doubleYoVariable);

      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      DoubleYoVariable transferSplitFraction = new DoubleYoVariable("transferSplitFraction1", registry);
      transferSplitFractions.add(transferSplitFraction);

      TransferEntryCMPMatrix transferEntryCMPMatrix = new TransferEntryCMPMatrix(swingSplitFractions, transferSplitFractions);

      Assert.assertEquals("", 4, transferEntryCMPMatrix.numRows);
      Assert.assertEquals("", 1, transferEntryCMPMatrix.numCols);
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

      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();
      DoubleYoVariable swingSplitFraction = new DoubleYoVariable("swingSplitFraction1", registry);
      swingSplitFractions.add(swingSplitFraction);

      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      DoubleYoVariable transferSplitFraction1 = new DoubleYoVariable("transferSplitFraction1", registry);
      DoubleYoVariable transferSplitFraction2 = new DoubleYoVariable("transferSplitFraction2", registry);
      transferSplitFractions.add(transferSplitFraction1);
      transferSplitFractions.add(transferSplitFraction2);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration1", registry));
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration2", registry));

      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      TransferEntryCMPMatrix transferEntryCMPMatrix = new TransferEntryCMPMatrix(swingSplitFractions, transferSplitFractions);

      for (int i = 0; i < iters; i++)
      {
         double transferRatio1 = 0.5 * random.nextDouble();
         double transferRatio2 = 0.5 * random.nextDouble();
         transferSplitFraction1.set(transferRatio1);
         transferSplitFraction2.set(transferRatio2);

         double swingRatio = 0.5 * random.nextDouble();
         swingSplitFraction.set(swingRatio);

         double doubleSupportDuration1 = 2.0 * random.nextDouble();
         double doubleSupportDuration2 = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration1);
         doubleSupportDurations.get(1).set(doubleSupportDuration2);

         double singleSupportDuration = 5.0 * random.nextDouble();
         singleSupportDurations.get(0).set(singleSupportDuration);


         boolean useTwoCMPs = false;
         transferEntryCMPMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega0);

         double currentTransferOnEntry = (1 - transferRatio1) * doubleSupportDuration1;
         double nextTransferOnEntry = transferRatio2 * doubleSupportDuration2;
         double timeOnEntry = currentTransferOnEntry + nextTransferOnEntry + singleSupportDuration;
         double projectionDuration = currentTransferOnEntry - timeOnEntry;
         double projection = Math.exp(omega0 * projectionDuration);

         shouldBe.zero();
         shouldBe.set(2, 0, 1.0 - projection);
         shouldBe.set(3, 0, -omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, transferEntryCMPMatrix, epsilon);

         shouldBe.zero();
         transferEntryCMPMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, transferEntryCMPMatrix, epsilon);

         useTwoCMPs = true;
         transferEntryCMPMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega0);

         double currentSwingOnEntry = swingRatio * singleSupportDuration;
         timeOnEntry = currentSwingOnEntry + currentTransferOnEntry;

         projectionDuration = currentTransferOnEntry - timeOnEntry;
         projection = Math.exp(omega0 * projectionDuration);

         shouldBe.zero();
         shouldBe.set(2, 0, 1.0 - projection);
         shouldBe.set(3, 0, -omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, transferEntryCMPMatrix, epsilon);

         shouldBe.zero();
         transferEntryCMPMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, transferEntryCMPMatrix, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationNoSteps()
   {
      DenseMatrix64F shouldBe = new DenseMatrix64F(4, 1);
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      Random random = new Random();
      int iters = 100;
      double omega0 = 3.0;

      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();
      DoubleYoVariable swingSplitFraction = new DoubleYoVariable("swingSplitFraction1", registry);
      swingSplitFractions.add(swingSplitFraction);

      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      DoubleYoVariable transferSplitFraction1 = new DoubleYoVariable("transferSplitFraction1", registry);
      DoubleYoVariable transferSplitFraction2 = new DoubleYoVariable("transferSplitFraction2", registry);
      transferSplitFractions.add(transferSplitFraction1);
      transferSplitFractions.add(transferSplitFraction2);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration1", registry));
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration2", registry));

      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      TransferEntryCMPMatrix transferEntryCMPMatrix = new TransferEntryCMPMatrix(swingSplitFractions, transferSplitFractions);

      for (int i = 0; i < iters; i++)
      {
         double transferRatio1 = 0.5 * random.nextDouble();
         double transferRatio2 = 0.5 * random.nextDouble();
         transferSplitFraction1.set(transferRatio1);
         transferSplitFraction2.set(transferRatio2);

         double swingRatio = 0.5 * random.nextDouble();
         swingSplitFraction.set(swingRatio);

         double doubleSupportDuration1 = 2.0 * random.nextDouble();
         double doubleSupportDuration2 = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration1);
         doubleSupportDurations.get(1).set(doubleSupportDuration2);

         double singleSupportDuration = 5.0 * random.nextDouble();
         singleSupportDurations.get(0).set(singleSupportDuration);


         boolean useTwoCMPs = false;
         transferEntryCMPMatrix.compute(0, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega0);

         double currentTransferOnEntry = (1 - transferRatio1) * doubleSupportDuration1;
         double projectionDuration = currentTransferOnEntry;
         double projection = Math.exp(omega0 * projectionDuration);

         shouldBe.zero();
         shouldBe.set(2, 0, 1.0 - projection);
         shouldBe.set(3, 0, -omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, transferEntryCMPMatrix, epsilon);

         shouldBe.zero();
         transferEntryCMPMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, transferEntryCMPMatrix, epsilon);

         useTwoCMPs = true;
         transferEntryCMPMatrix.compute(0, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega0);

         projectionDuration = currentTransferOnEntry;
         projection = Math.exp(omega0 * projectionDuration);

         shouldBe.zero();
         shouldBe.set(2, 0, 1.0 - projection);
         shouldBe.set(3, 0, -omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, transferEntryCMPMatrix, epsilon);

         shouldBe.zero();
         transferEntryCMPMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, transferEntryCMPMatrix, epsilon);
      }
   }
}
