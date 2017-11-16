package us.ihmc.commonWalkingControlModules.capturePoint.optimization.recursiveController.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.multipliers.stateMatrices.swing.SwingInitialICPMatrix;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.InterpolationTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.testing.JUnitTools;

import java.util.ArrayList;
import java.util.Random;

public class SwingInitialICPMatrixTest
{
   private static final double epsilon = 0.00001;
   private static final double minimumBlendingTime = 0.05;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      YoDouble startOfSplineTime = new YoDouble("startOfSplineTime", registry);

      SwingInitialICPMatrix initialICPMatrix = new SwingInitialICPMatrix(startOfSplineTime, false, minimumBlendingTime);

      Assert.assertEquals("", 4, initialICPMatrix.numRows);
      Assert.assertEquals("", 1, initialICPMatrix.numCols);
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

      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new YoDouble("currentDoubleSupportDuration", registry));
      doubleSupportDurations.add(new YoDouble("upcomingDoubleSupportDuration", registry));
      singleSupportDurations.add(new YoDouble("singleSupportDuration", registry));

      SwingInitialICPMatrix initialICPMatrix = new SwingInitialICPMatrix(startOfSplineTime, false, minimumBlendingTime);

      for (int i = 0; i < iters; i++)
      {
         double doubleSupportDuration = 2.0 * random.nextDouble();
         double upcomingDoubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);
         doubleSupportDurations.get(1).set(upcomingDoubleSupportDuration);
         singleSupportDurations.get(0).set(singleSupportDuration);

         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;

         startOfSplineTime.set(startOfSpline);
         String name = "doubleSupportDuration = " + doubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;


         double projection = Math.exp(omega0 * startOfSpline);

         initialICPMatrix.compute(omega0);

         shouldBe.zero();
         shouldBe.set(0, 0, projection);
         shouldBe.set(1, 0, omega0 * projection);

         JUnitTools.assertMatrixEquals(name, shouldBe, initialICPMatrix, epsilon);

         shouldBe.zero();
         initialICPMatrix.reset();
         JUnitTools.assertMatrixEquals(name, shouldBe, initialICPMatrix, epsilon);
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

      ArrayList<YoDouble> doubleSupportDurations = new ArrayList<>();
      ArrayList<YoDouble> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new YoDouble("currentDoubleSupportDuration", registry));
      doubleSupportDurations.add(new YoDouble("upcomingDoubleSupportDuration", registry));
      singleSupportDurations.add(new YoDouble("singleSupportDuration", registry));

      SwingInitialICPMatrix initialICPMatrix = new SwingInitialICPMatrix(startOfSplineTime, true, minimumBlendingTime);

      for (int i = 0; i < iters; i++)
      {
         double doubleSupportDuration = 2.0 * random.nextDouble();
         double upcomingDoubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();
         doubleSupportDurations.get(0).set(doubleSupportDuration);
         doubleSupportDurations.get(1).set(upcomingDoubleSupportDuration);
         singleSupportDurations.get(0).set(singleSupportDuration);

         double minimumSplineTime = Math.min(singleSupportDuration, 0.5);
         double startOfSpline = 0.2 * random.nextDouble();
         double endOfSpline = singleSupportDuration - 0.2 * random.nextDouble();
         if (minimumSplineTime > endOfSpline - startOfSpline)
            startOfSpline = 0.0;

         startOfSplineTime.set(startOfSpline);
         String name = "doubleSupportDuration = " + doubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;

         initialICPMatrix.compute(omega0);

         shouldBe.zero();

         if (startOfSpline < minimumBlendingTime )
         {
            double projection = Math.exp(omega0 * startOfSpline);
            projection = InterpolationTools.linearInterpolate(projection, 0.0, startOfSpline / minimumBlendingTime);
            shouldBe.set(0, 0, projection);
            shouldBe.set(1, 0, omega0 * projection);
         }

         JUnitTools.assertMatrixEquals(name, shouldBe, initialICPMatrix, epsilon);

         shouldBe.zero();
         initialICPMatrix.reset();
         JUnitTools.assertMatrixEquals(name, shouldBe, initialICPMatrix, epsilon);
      }
   }
}
