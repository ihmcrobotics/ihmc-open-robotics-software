package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingEntryCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.MutationTestingTools;

import java.util.ArrayList;
import java.util.Random;

public class NewSwingEntryCMPMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);

      NewSwingEntryCMPMatrix swingEntryCMPMatrix = new NewSwingEntryCMPMatrix(startOfSplineTime);

      Assert.assertEquals("", 4, swingEntryCMPMatrix.numRows);
      Assert.assertEquals("", 1, swingEntryCMPMatrix.numCols);
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
      DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable("exitCMPDurationInPercentOfStepTime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      NewSwingEntryCMPMatrix swingEntryCMPMatrix = new NewSwingEntryCMPMatrix(startOfSplineTime);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.5 * random.nextDouble();
         double exitRatio = 0.5 * random.nextDouble();
         double startOfSpline = 0.2 * random.nextDouble();

         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfStepTime.set(exitRatio);
         startOfSplineTime.set(startOfSpline);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);
         singleSupportDurations.get(0).set(singleSupportDuration);

         String name = "splitRatio = " + splitRatio + ", exitRatio = " + exitRatio + ",\n doubleSupportDuration = " + doubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;

         double projectionTime = startOfSpline;
         double projection = Math.exp(omega0 * projectionTime);

         swingEntryCMPMatrix.compute(omega0);

         shouldBe.zero();
         shouldBe.set(0, 0, 1.0 - projection);
         shouldBe.set(1, 0, -omega0 * projection);

         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPMatrix, epsilon);

         swingEntryCMPMatrix.reset();
         swingEntryCMPMatrix.compute(omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPMatrix, epsilon);

         shouldBe.zero();
         swingEntryCMPMatrix.reset();
         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPMatrix, epsilon);
      }
   }
}
