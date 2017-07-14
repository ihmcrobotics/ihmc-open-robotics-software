package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.testing.JUnitTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class SwingExitCMPMatrixTest
{
   private static final double epsilon = 0.00001;
   private static final double minimumBlendingTime = 0.05;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      List<YoDouble> swingSplitFractions = new ArrayList<>();
      YoDouble swingSplitFraction = new YoDouble("swingSplitFraction", registry);
      swingSplitFractions.add(swingSplitFraction);

      List<YoDouble> transferSplitFractions = new ArrayList<>();
      YoDouble transferSplitFraction = new YoDouble("transferSplitFraction", registry);
      transferSplitFractions.add(transferSplitFraction);

      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);

      SwingExitCMPMatrix swingExitCMPMatrix = new SwingExitCMPMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, false,
            minimumBlendingTime);

      Assert.assertEquals("", 4, swingExitCMPMatrix.numRows);
      Assert.assertEquals("", 1, swingExitCMPMatrix.numCols);
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

      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(new YoDouble("singleSupportDuration", registry));
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new YoDouble("doubleSupportDuration1", registry));
      doubleSupportDurations.add(new YoDouble("doubleSupportDuration2", registry));

      List<YoDouble> swingSplitFractions = new ArrayList<>();
      YoDouble swingSplitFraction = new YoDouble("swingSplitFraction", registry);
      swingSplitFractions.add(swingSplitFraction);

      List<YoDouble> transferSplitFractions = new ArrayList<>();
      YoDouble transferSplitFraction1 = new YoDouble("transferSplitFraction1", registry);
      YoDouble transferSplitFraction2 = new YoDouble("transferSplitFraction2", registry);
      transferSplitFractions.add(transferSplitFraction1);
      transferSplitFractions.add(transferSplitFraction2);

      SwingExitCMPMatrix swingExitCMPMatrix = new SwingExitCMPMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, false,
            minimumBlendingTime);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.7 * random.nextDouble();
         swingSplitFraction.set(splitRatio);

         double transferRatio1 = 0.7 * random.nextDouble();
         double transferRatio2 = 0.7 * random.nextDouble();
         transferSplitFraction1.set(transferRatio1);
         transferSplitFraction2.set(transferRatio2);

         double singleSupportDuration = 5.0 * random.nextDouble();
         singleSupportDurations.get(0).set(singleSupportDuration);

         double doubleSupportDuration = 5.0 * random.nextDouble();
         double nextDoubleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);
         doubleSupportDurations.get(1).set(nextDoubleSupportDuration);

         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         double currentSwingOnEntry = splitRatio * singleSupportDuration;
         double currentSwingOnExit = (1.0 - splitRatio) * singleSupportDuration;
         double nextTransferOnExit = transferRatio2 * nextDoubleSupportDuration;
         double timeOnExit = currentSwingOnExit + nextTransferOnExit;
         double currentSplineOnExit = endOfSpline - currentSwingOnEntry;

         double projectionTime = currentSplineOnExit - timeOnExit;
         double projection = Math.exp(omega0 * projectionTime);

         swingExitCMPMatrix.compute(singleSupportDurations, doubleSupportDurations, omega0);

         shouldBe.zero();
         shouldBe.set(2, 0, 1.0 - projection);
         shouldBe.set(3, 0, -omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, swingExitCMPMatrix, epsilon);

         swingExitCMPMatrix.reset();
         swingExitCMPMatrix.compute(singleSupportDurations, doubleSupportDurations, omega0);
         JUnitTools.assertMatrixEquals(shouldBe, swingExitCMPMatrix, epsilon);

         shouldBe.zero();
         swingExitCMPMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, swingExitCMPMatrix, epsilon);
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

      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(new YoDouble("singleSupportDuration", registry));
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new YoDouble("doubleSupportDuration1", registry));
      doubleSupportDurations.add(new YoDouble("doubleSupportDuration2", registry));

      List<YoDouble> swingSplitFractions = new ArrayList<>();
      YoDouble swingSplitFraction = new YoDouble("swingSplitFraction", registry);
      swingSplitFractions.add(swingSplitFraction);

      List<YoDouble> transferSplitFractions = new ArrayList<>();
      YoDouble transferSplitFraction1 = new YoDouble("transferSplitFraction1", registry);
      YoDouble transferSplitFraction2 = new YoDouble("transferSplitFraction2", registry);
      transferSplitFractions.add(transferSplitFraction1);
      transferSplitFractions.add(transferSplitFraction2);

      SwingExitCMPMatrix swingExitCMPMatrix = new SwingExitCMPMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, true,
            minimumBlendingTime);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.7 * random.nextDouble();
         swingSplitFraction.set(splitRatio);

         double transferRatio1 = 0.7 * random.nextDouble();
         double transferRatio2 = 0.7 * random.nextDouble();
         transferSplitFraction1.set(transferRatio1);
         transferSplitFraction2.set(transferRatio2);

         double singleSupportDuration = 5.0 * random.nextDouble();
         singleSupportDurations.get(0).set(singleSupportDuration);

         double doubleSupportDuration = 5.0 * random.nextDouble();
         double nextDoubleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);
         doubleSupportDurations.get(1).set(nextDoubleSupportDuration);

         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);

         double currentSwingOnEntry = splitRatio * singleSupportDuration;
         double currentSwingOnExit = (1.0 - splitRatio) * singleSupportDuration;
         double nextTransferOnExit = transferRatio2 * nextDoubleSupportDuration;
         double timeOnExit = currentSwingOnExit + nextTransferOnExit;
         double currentSplineOnExit = endOfSpline - currentSwingOnEntry;
         double currentSplineOnEntry = currentSwingOnEntry - startOfSpline;

         double projectionTime = currentSplineOnExit - timeOnExit;
         double projection = Math.exp(omega0 * projectionTime);

         double initialProjection;

         if (startOfSpline >= minimumBlendingTime)
            initialProjection = Math.exp(-omega0 * currentSplineOnEntry) * (1.0 - Math.exp(-omega0 * timeOnExit));
         else
         {
            double recursionMultiplier = Math.exp(-omega0 * currentSplineOnEntry) * (1.0 - Math.exp(-omega0 * timeOnExit));
            double projectionMultiplier = 0.0;

            initialProjection = InterpolationTools.linearInterpolate(projectionMultiplier, recursionMultiplier, startOfSpline / minimumBlendingTime);
         }

         swingExitCMPMatrix.compute(singleSupportDurations, doubleSupportDurations, omega0);

         shouldBe.zero();
         shouldBe.set(0, 0, initialProjection);
         shouldBe.set(1, 0, omega0 * initialProjection);
         shouldBe.set(2, 0, 1.0 - projection);
         shouldBe.set(3, 0, -omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, swingExitCMPMatrix, epsilon);

         swingExitCMPMatrix.reset();
         swingExitCMPMatrix.compute(singleSupportDurations, doubleSupportDurations, omega0);
         JUnitTools.assertMatrixEquals(shouldBe, swingExitCMPMatrix, epsilon);

         shouldBe.zero();
         swingExitCMPMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, swingExitCMPMatrix, epsilon);
      }
   }
}
