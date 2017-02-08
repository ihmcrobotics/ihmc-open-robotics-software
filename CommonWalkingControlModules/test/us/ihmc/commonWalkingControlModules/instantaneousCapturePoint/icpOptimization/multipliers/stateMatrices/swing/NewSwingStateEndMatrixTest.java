package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

import java.util.ArrayList;
import java.util.Random;

public class NewSwingStateEndMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable("exitCMPDurationInPercentOfStepTime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);

      NewSwingStateEndMatrix swingStateEndMatrix = new NewSwingStateEndMatrix(doubleSupportSplitRatio,
            exitCMPDurationInPercentOfStepTime, startOfSplineTime);

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

      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable("exitCMPDurationInPercentOfStepTime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      ArrayList<DoubleYoVariable> doubleSupportDurations = new ArrayList<>();
      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      doubleSupportDurations.add(new DoubleYoVariable("currentDoubleSupportDuration", registry));
      doubleSupportDurations.add(new DoubleYoVariable("upcomingDoubleSupportDuration", registry));
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      NewSwingStateEndMatrix swingStateEndMatrix = new NewSwingStateEndMatrix(doubleSupportSplitRatio, exitCMPDurationInPercentOfStepTime, endOfSplineTime);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.5 * random.nextDouble();
         double exitRatio = 0.5 * random.nextDouble();

         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfStepTime.set(exitRatio);

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
         if (minimumSplineTime > endOfSpline - startOfSpline)
            endOfSpline = singleSupportDuration;

         startOfSplineTime.set(startOfSpline);
         endOfSplineTime.set(endOfSpline);
         totalTrajectoryTime.set(singleSupportDuration);
         String name = "splitRatio = " + splitRatio + ", exitRatio = " + exitRatio + ",\n doubleSupportDuration = " + doubleSupportDuration + ", singleSupportDuration = " + singleSupportDuration;

         double upcomingInitialDoubleSupportDuration = splitRatio * upcomingDoubleSupportDuration;
         double timeSpentOnExitCMP = exitRatio * (singleSupportDuration + doubleSupportDuration);

         double projectionTime = endOfSpline - singleSupportDuration + timeSpentOnExitCMP - upcomingInitialDoubleSupportDuration;
         double projection = Math.exp(omega0 * projectionTime);

         swingStateEndMatrix.compute(doubleSupportDurations, singleSupportDurations, omega0);

         shouldBe.zero();
         shouldBe.set(2, 0, projection);
         shouldBe.set(3, 0, omega0 * projection);

         JUnitTools.assertMatrixEquals(name, shouldBe, swingStateEndMatrix, epsilon);

         shouldBe.zero();
         swingStateEndMatrix.reset();
         JUnitTools.assertMatrixEquals(name, shouldBe, swingStateEndMatrix, epsilon);
      }
   }
}
