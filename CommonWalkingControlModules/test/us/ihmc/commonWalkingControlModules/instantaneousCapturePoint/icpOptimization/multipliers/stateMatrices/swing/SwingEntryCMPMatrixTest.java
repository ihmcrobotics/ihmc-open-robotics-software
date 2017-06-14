package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.DoubleYoVariable;
import us.ihmc.robotics.testing.JUnitTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class SwingEntryCMPMatrixTest
{
   private static final double epsilon = 0.00001;
   private static final double minimumBlendingTime = 0.05;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();

      SwingEntryCMPMatrix swingEntryCMPMatrix = new SwingEntryCMPMatrix(swingSplitFractions, startOfSplineTime, false, minimumBlendingTime);

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

      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);

      ArrayList<DoubleYoVariable> swingSplitFractions = new ArrayList<>();
      swingSplitFractions.add(new DoubleYoVariable("swingSplitFraction", registry));

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      SwingEntryCMPMatrix swingEntryCMPMatrix = new SwingEntryCMPMatrix(swingSplitFractions, startOfSplineTime, false, minimumBlendingTime);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.5 * random.nextDouble();
         double startOfSpline = 0.2 * random.nextDouble();

         startOfSplineTime.set(startOfSpline);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);
         singleSupportDurations.get(0).set(singleSupportDuration);

         String name = "splitRatio = " + splitRatio + ",\n doubleSupportDuration = " + doubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;

         double projectionTime = startOfSpline;
         double projection = Math.exp(omega0 * projectionTime);

         swingEntryCMPMatrix.compute(singleSupportDurations, omega0);

         shouldBe.zero();
         shouldBe.set(0, 0, 1.0 - projection);
         shouldBe.set(1, 0, -omega0 * projection);

         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPMatrix, epsilon);

         swingEntryCMPMatrix.reset();
         swingEntryCMPMatrix.compute(singleSupportDurations, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPMatrix, epsilon);

         shouldBe.zero();
         swingEntryCMPMatrix.reset();
         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPMatrix, epsilon);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculationWithBlending()
   {
      DenseMatrix64F shouldBe = new DenseMatrix64F(4, 1);
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      Random random = new Random();
      int iters = 100;
      double omega0 = 3.0;

      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);

      ArrayList<DoubleYoVariable> swingSplitFractions = new ArrayList<>();
      swingSplitFractions.add(new DoubleYoVariable("swingSplitFraction", registry));

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("doubleSupportDuration", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      SwingEntryCMPMatrix swingEntryCMPMatrix = new SwingEntryCMPMatrix(swingSplitFractions, startOfSplineTime, true, minimumBlendingTime);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.5 * random.nextDouble();
         double startOfSpline = 0.2 * random.nextDouble();

         swingSplitFractions.get(0).set(splitRatio);
         startOfSplineTime.set(startOfSpline);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);
         singleSupportDurations.get(0).set(singleSupportDuration);

         String name = "splitRatio = " + splitRatio + ",\n doubleSupportDuration = " + doubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;

         double swingOnEntry = splitRatio * singleSupportDuration;
         double splineOnEntry = swingOnEntry - startOfSpline;

         double recursionMultiplier = 1.0 - Math.exp(-omega0 * splineOnEntry);
         double projectionMultiplier = 1.0 - Math.exp(omega0 * startOfSpline);

         double projection;

         if (startOfSpline >= minimumBlendingTime)
         {
            projection = recursionMultiplier;
         }
         else
         {
            projection = InterpolationTools.linearInterpolate(projectionMultiplier, recursionMultiplier, startOfSpline / minimumBlendingTime);
         }

         swingEntryCMPMatrix.compute(singleSupportDurations, omega0);

         shouldBe.zero();
         shouldBe.set(0, 0, projection);
         shouldBe.set(1, 0, omega0 * (projection - 1.0));

         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPMatrix, epsilon);

         swingEntryCMPMatrix.reset();
         swingEntryCMPMatrix.compute(singleSupportDurations, omega0);
         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPMatrix, epsilon);

         shouldBe.zero();
         swingEntryCMPMatrix.reset();
         JUnitTools.assertMatrixEquals(name, shouldBe, swingEntryCMPMatrix, epsilon);
      }
   }
}
