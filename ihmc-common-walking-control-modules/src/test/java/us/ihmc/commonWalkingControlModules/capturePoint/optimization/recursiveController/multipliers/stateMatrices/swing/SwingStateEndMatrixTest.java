package us.ihmc.commonWalkingControlModules.capturePoint.optimization.recursiveController.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.swing.SwingStateEndMatrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.testing.JUnitTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class SwingStateEndMatrixTest
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


      SwingStateEndMatrix swingStateEndMatrix = new SwingStateEndMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, false,
            minimumBlendingTime);

      Assert.assertEquals("", 4, swingStateEndMatrix.numRows);
      Assert.assertEquals("", 1, swingStateEndMatrix.numCols);
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

      List<YoDouble> swingSplitFractions = new ArrayList<>();
      YoDouble swingSplitFraction = new YoDouble("swingSplitFraction", registry);
      swingSplitFractions.add(swingSplitFraction);

      List<YoDouble> transferSplitFractions = new ArrayList<>();
      YoDouble transferSplitFraction1 = new YoDouble("transferSplitFraction1", registry);
      YoDouble transferSplitFraction2 = new YoDouble("transferSplitFraction2", registry);
      transferSplitFractions.add(transferSplitFraction1);
      transferSplitFractions.add(transferSplitFraction2);

      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(new YoDouble("singleSupportDuration", registry));
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new YoDouble("doubleSupportDuration1", registry));
      doubleSupportDurations.add(new YoDouble("doubleSupportDuration2", registry));

      SwingStateEndMatrix swingStateEndMatrix = new SwingStateEndMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, false,
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

         double doubleSupportDuration1 = 2.0 * random.nextDouble();
         double doubleSupportDuration2 = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration1);
         doubleSupportDurations.get(1).set(doubleSupportDuration2);

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
         double nextTransferOnExit = transferRatio2 * doubleSupportDuration2;
         double timeOnExit = currentSwingOnExit + nextTransferOnExit;
         double currentSplineOnExit = endOfSpline - currentSwingOnEntry;

         double projectionTime = currentSplineOnExit - timeOnExit;
         double projection = Math.exp(omega0 * projectionTime);

         swingStateEndMatrix.compute(singleSupportDurations, doubleSupportDurations, omega0);

         shouldBe.zero();
         shouldBe.set(2, 0, projection);
         shouldBe.set(3, 0, omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, swingStateEndMatrix, epsilon);

         shouldBe.zero();
         swingStateEndMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, swingStateEndMatrix, epsilon);
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

      List<YoDouble> swingSplitFractions = new ArrayList<>();
      YoDouble swingSplitFraction = new YoDouble("swingSplitFraction", registry);
      swingSplitFractions.add(swingSplitFraction);

      List<YoDouble> transferSplitFractions = new ArrayList<>();
      YoDouble transferSplitFraction1 = new YoDouble("transferSplitFraction1", registry);
      YoDouble transferSplitFraction2 = new YoDouble("transferSplitFraction2", registry);
      transferSplitFractions.add(transferSplitFraction1);
      transferSplitFractions.add(transferSplitFraction2);

      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);
      YoDouble endOfSplineTime = new YoDouble("endOfSplineTime", registry);
      YoDouble totalTrajectoryTime = new YoDouble("totalTrajectoryTime", registry);

      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(new YoDouble("singleSupportDuration", registry));
      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new YoDouble("doubleSupportDuration1", registry));
      doubleSupportDurations.add(new YoDouble("doubleSupportDuration2", registry));

      SwingStateEndMatrix swingStateEndMatrix = new SwingStateEndMatrix(swingSplitFractions, transferSplitFractions, startOfSplineTime, endOfSplineTime, true,
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

         double doubleSupportDuration1 = 2.0 * random.nextDouble();
         double doubleSupportDuration2 = 2.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration1);
         doubleSupportDurations.get(1).set(doubleSupportDuration2);

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
         double nextTransferOnExit = transferRatio2 * doubleSupportDuration2;
         double timeOnExit = currentSwingOnExit + nextTransferOnExit;
         double currentSplineOnExit = endOfSpline - currentSwingOnEntry;
         double currentSplineOnEntry = currentSwingOnEntry - startOfSpline;

         double projectionTime = currentSplineOnExit - timeOnExit;
         double projection = Math.exp(omega0 * projectionTime);

         double initialProjectionTime = timeOnExit + currentSplineOnEntry;
         double initialProjection;

         if (startOfSpline >= minimumBlendingTime)
         {
            initialProjection = Math.exp(-omega0 * initialProjectionTime);
         }
         else
         {
            double recursion =  Math.exp(-omega0 * initialProjectionTime);

            initialProjection = InterpolationTools.linearInterpolate(0.0, recursion, startOfSpline / minimumBlendingTime);
         }

         swingStateEndMatrix.compute(singleSupportDurations, doubleSupportDurations, omega0);

         shouldBe.zero();
         shouldBe.set(0, 0, initialProjection);
         shouldBe.set(1, 0, omega0 * initialProjection);
         shouldBe.set(2, 0, projection);
         shouldBe.set(3, 0, omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, swingStateEndMatrix, epsilon);

         shouldBe.zero();
         swingStateEndMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, swingStateEndMatrix, epsilon);
      }
   }
}
