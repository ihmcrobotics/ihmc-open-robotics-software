package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.testing.JUnitTools;

public class TransferEntryCMPMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      DoubleYoVariable doubleYoVariable = new DoubleYoVariable("doubleYoVariable", registry);
      transferSplitFractions.add(doubleYoVariable);

      TransferEntryCMPMatrix transferEntryCMPMatrix = new TransferEntryCMPMatrix(transferSplitFractions);

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

      List<DoubleYoVariable> transferSplitFractions = new ArrayList<>();
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      transferSplitFractions.add(doubleSupportSplitRatio);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration", registry));

      TransferEntryCMPMatrix transferEntryCMPMatrix = new TransferEntryCMPMatrix(transferSplitFractions);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.5 * random.nextDouble();

         doubleSupportSplitRatio.set(splitRatio);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);

         double endOfDoubleSupportDuration = (1 - splitRatio) * doubleSupportDuration;
         double projection = Math.exp(omega0 * endOfDoubleSupportDuration);

         transferEntryCMPMatrix.compute(doubleSupportDurations, omega0);

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
