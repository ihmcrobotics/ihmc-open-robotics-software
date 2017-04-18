package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.multipliers.stateMatrices.swing;

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

public class SwingStateEndMatrixTest
{
   private static final double epsilon = 0.00001;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();
      DoubleYoVariable swingSplitFraction = new DoubleYoVariable("swingSplitFraction", registry);
      swingSplitFractions.add(swingSplitFraction);

      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);


      SwingStateEndMatrix swingStateEndMatrix = new SwingStateEndMatrix(swingSplitFractions, startOfSplineTime);

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

      List<DoubleYoVariable> swingSplitFractions = new ArrayList<>();
      DoubleYoVariable swingSplitFraction = new DoubleYoVariable("swingSplitFraction", registry);
      swingSplitFractions.add(swingSplitFraction);

      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      ArrayList<DoubleYoVariable> singleSupportDurations = new ArrayList<>();
      singleSupportDurations.add(new DoubleYoVariable("singleSupportDuration", registry));

      SwingStateEndMatrix swingStateEndMatrix = new SwingStateEndMatrix(swingSplitFractions, endOfSplineTime);

      for (int i = 0; i < iters; i++)
      {
         double splitRatio = 0.7 * random.nextDouble();
         swingSplitFraction.set(splitRatio);

         double singleSupportDuration = 5.0 * random.nextDouble();
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

         double swingTimeSpentOnEntryCMP =  splitRatio * singleSupportDuration;

         double projectionTime = endOfSpline - swingTimeSpentOnEntryCMP;
         double projection = Math.exp(omega0 * projectionTime);

         swingStateEndMatrix.compute(singleSupportDurations, omega0);

         shouldBe.zero();
         shouldBe.set(2, 0, projection);
         shouldBe.set(3, 0, omega0 * projection);

         JUnitTools.assertMatrixEquals(shouldBe, swingStateEndMatrix, epsilon);

         shouldBe.zero();
         swingStateEndMatrix.reset();
         JUnitTools.assertMatrixEquals(shouldBe, swingStateEndMatrix, epsilon);
      }
   }
}
