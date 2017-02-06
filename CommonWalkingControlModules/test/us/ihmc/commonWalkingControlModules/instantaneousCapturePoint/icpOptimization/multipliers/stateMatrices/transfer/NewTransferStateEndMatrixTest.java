package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.transfer;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing.NewSwingStateEndMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

import java.util.ArrayList;
import java.util.Random;

public class NewTransferStateEndMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);

      NewTransferStateEndMatrix stateEndMatrix = new NewTransferStateEndMatrix(doubleSupportSplitRatio);

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

      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("currentDoubleSupportDuration", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      NewTransferStateEndMatrix stateEndMatrix = new NewTransferStateEndMatrix(doubleSupportSplitRatio);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.5 * random.nextDouble();

         doubleSupportSplitRatio.set(splitRatio);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);
         singleSupportDurations.get(0).set(singleSupportDuration);

         String name = "splitRatio = " + splitRatio + ",\n doubleSupportDuration = " + doubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;

         double projectionTime = (1.0 - splitRatio) * doubleSupportDuration;
         double projection = Math.exp(omega0 * projectionTime);

         boolean useTwoCMPs = true;

         stateEndMatrix.compute(doubleSupportDurations, omega0, useTwoCMPs);

         shouldBe.zero();
         shouldBe.set(2, 0, projection);
         shouldBe.set(3, 0, omega0 * projection);

         JUnitTools.assertMatrixEquals(name, shouldBe, stateEndMatrix, epsilon);

         shouldBe.zero();
         stateEndMatrix.reset();
         JUnitTools.assertMatrixEquals(name, shouldBe, stateEndMatrix, epsilon);
      }
   }
}
