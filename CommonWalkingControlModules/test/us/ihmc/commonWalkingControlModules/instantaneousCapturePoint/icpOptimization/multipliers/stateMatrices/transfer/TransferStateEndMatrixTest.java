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

public class TransferStateEndMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();
      DoubleYoVariable singleSupportSplitRatio = new DoubleYoVariable("singleSupportSplitRatio", registry);
      swingSplitFractions.add(singleSupportSplitRatio);
      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      transferSplitFractions.add(doubleSupportSplitRatio);

      TransferStateEndMatrix stateEndMatrix = new TransferStateEndMatrix(swingSplitFractions, transferSplitFractions);

      Assert.assertEquals("", 4, stateEndMatrix.numRows);
      Assert.assertEquals("", 1, stateEndMatrix.numCols);
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
      DoubleYoVariable swingSplitFraction1 = new DoubleYoVariable("swingSplitFraction1", registry);
      swingSplitFractions.add(swingSplitFraction1);

      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      DoubleYoVariable transferSplitFraction1 = new DoubleYoVariable("transferSplitFraction1", registry);
      DoubleYoVariable transferSplitFraction2 = new DoubleYoVariable("transferSplitFraction2", registry);
      transferSplitFractions.add(transferSplitFraction1);
      transferSplitFractions.add(transferSplitFraction2);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration1", registry));
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration2", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      TransferStateEndMatrix stateEndMatrix = new TransferStateEndMatrix(swingSplitFractions, transferSplitFractions);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio1 = 0.5 * random.nextDouble();
         double splitRatio2 = 0.5 * random.nextDouble();
         transferSplitFraction1.set(splitRatio1);
         transferSplitFraction2.set(splitRatio2);

         double swingRatio = 0.5 * random.nextDouble();
         swingSplitFraction1.set(swingRatio);

         double doubleSupportDuration1 = 2.0 * random.nextDouble();
         double doubleSupportDuration2 = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration1);
         doubleSupportDurations.get(1).set(doubleSupportDuration2);
         singleSupportDurations.get(0).set(singleSupportDuration);


         boolean useTwoCMPs = true;
         stateEndMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega0);

         double transferOnEntry = (1.0 - splitRatio1) * doubleSupportDuration1;
         double swingOnEntry = swingRatio * singleSupportDuration;
         double swingOnExit = (1.0 - swingRatio) * singleSupportDuration;
         double nextTransferOnExit = splitRatio2 * doubleSupportDuration2;
         double timeOnEntry = transferOnEntry + swingOnEntry;
         double timeOnExit = swingOnExit + nextTransferOnExit;

         double projection = Math.exp(omega0 * (transferOnEntry - timeOnEntry - timeOnExit));

         shouldBe.zero();
         shouldBe.set(2, 0, projection);
         shouldBe.set(3, 0, omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, stateEndMatrix, epsilon);

         shouldBe.zero();
         stateEndMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, stateEndMatrix, epsilon);


         useTwoCMPs = false;
         stateEndMatrix.compute(1, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega0);

         double timeOnCMP = nextTransferOnExit + singleSupportDuration + transferOnEntry;
         projection = Math.exp(omega0 * (transferOnEntry - timeOnCMP));

         shouldBe.zero();
         shouldBe.set(2, 0, projection);
         shouldBe.set(3, 0, omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, stateEndMatrix, epsilon);

         shouldBe.zero();
         stateEndMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, stateEndMatrix, epsilon);
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
      DoubleYoVariable swingSplitFraction1 = new DoubleYoVariable("swingSplitFraction1", registry);
      swingSplitFractions.add(swingSplitFraction1);

      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      DoubleYoVariable transferSplitFraction1 = new DoubleYoVariable("transferSplitFraction1", registry);
      DoubleYoVariable transferSplitFraction2 = new DoubleYoVariable("transferSplitFraction2", registry);
      transferSplitFractions.add(transferSplitFraction1);
      transferSplitFractions.add(transferSplitFraction2);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration1", registry));
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration2", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      TransferStateEndMatrix stateEndMatrix = new TransferStateEndMatrix(swingSplitFractions, transferSplitFractions);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio1 = 0.5 * random.nextDouble();
         double splitRatio2 = 0.5 * random.nextDouble();
         transferSplitFraction1.set(splitRatio1);
         transferSplitFraction2.set(splitRatio2);

         double swingRatio = 0.5 * random.nextDouble();
         swingSplitFraction1.set(swingRatio);

         double doubleSupportDuration1 = 2.0 * random.nextDouble();
         double doubleSupportDuration2 = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration1);
         doubleSupportDurations.get(1).set(doubleSupportDuration2);
         singleSupportDurations.get(0).set(singleSupportDuration);


         boolean useTwoCMPs = true;
         stateEndMatrix.compute(0, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega0);

         double transferOnEntry = (1.0 - splitRatio1) * doubleSupportDuration1;
         double nextTransferOnExit = splitRatio2 * doubleSupportDuration2;

         double projection = Math.exp(omega0 * transferOnEntry);

         shouldBe.zero();
         shouldBe.set(2, 0, projection);
         shouldBe.set(3, 0, omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, stateEndMatrix, epsilon);

         shouldBe.zero();
         stateEndMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, stateEndMatrix, epsilon);


         useTwoCMPs = false;
         stateEndMatrix.compute(0, singleSupportDurations, doubleSupportDurations, useTwoCMPs, omega0);

         projection = Math.exp(omega0 * transferOnEntry);

         shouldBe.zero();
         shouldBe.set(2, 0, projection);
         shouldBe.set(3, 0, omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, stateEndMatrix, epsilon);

         shouldBe.zero();
         stateEndMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, stateEndMatrix, epsilon);
      }
   }
}
