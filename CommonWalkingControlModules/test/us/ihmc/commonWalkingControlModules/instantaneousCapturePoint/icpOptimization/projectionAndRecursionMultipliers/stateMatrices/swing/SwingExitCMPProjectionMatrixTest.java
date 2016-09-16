package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.projectionAndRecursionMultipliers.stateMatrices.swing.SwingExitCMPProjectionMatrix;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.util.Random;

public class SwingExitCMPProjectionMatrixTest
{
   private static final double epsilon = 0.00001;

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCreationSize()
   {
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      SwingExitCMPProjectionMatrix swingExitCMPProjectionMatrix = new SwingExitCMPProjectionMatrix(doubleSupportSplitRatio, exitCMPDurationInPercentOfSteptime,
            startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      Assert.assertEquals("", 4, swingExitCMPProjectionMatrix.numRows);
      Assert.assertEquals("", 1, swingExitCMPProjectionMatrix.numCols);
   }

   @DeployableTestMethod(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCalculation()
   {
      DenseMatrix64F shouldBe = new DenseMatrix64F(4, 1);
      YoVariableRegistry registry = new YoVariableRegistry("registry");

      Random random = new Random();
      int iters = 100;



      DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
      DoubleYoVariable doubleSupportSplitRatio = new DoubleYoVariable("doubleSupportSplitRatio", registry);
      DoubleYoVariable exitCMPDurationInPercentOfSteptime = new DoubleYoVariable("exitCMPDurationInPercentOfSteptime", registry);
      DoubleYoVariable startOfSplineTime = new DoubleYoVariable("startOfSplineTime", registry);
      DoubleYoVariable endOfSplineTime = new DoubleYoVariable("endOfSplineTime", registry);
      DoubleYoVariable totalTrajectoryTime = new DoubleYoVariable("totalTrajectoryTime", registry);

      SwingExitCMPProjectionMatrix swingExitCMPProjectionMatrix = new SwingExitCMPProjectionMatrix(doubleSupportSplitRatio, exitCMPDurationInPercentOfSteptime,
            startOfSplineTime, endOfSplineTime, totalTrajectoryTime);

      for (int i = 0; i < iters; i++)
      {
         double omega0 = 3.0;
         double splitRatio = 0.5 * random.nextDouble();
         double exitRatio = 0.5 * random.nextDouble();

         omega.set(omega0);
         doubleSupportSplitRatio.set(splitRatio);
         exitCMPDurationInPercentOfSteptime.set(exitRatio);

         double doubleSupportDuration = 2.0 * random.nextDouble();
         double upcomingDoubleSupportDuration = 2.0 * random.nextDouble();
         double singleSupportDuration = 5.0 * random.nextDouble();

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

         double stepDuration = singleSupportDuration + doubleSupportDuration;
         double upcomingInitialDoubleSupport = splitRatio * upcomingDoubleSupportDuration;
         double endOfDoubleSupport = (1.0 - splitRatio) * doubleSupportDuration;

         double timeOnExit = exitRatio * (singleSupportDuration + doubleSupportDuration);
         double timeOnEntry = (1.0 - exitRatio) * (singleSupportDuration + doubleSupportDuration);

         double thirdSegmentTime = singleSupportDuration - endOfSpline;
         double firstSegmentTime = startOfSpline;

         double initialMultiplier = Math.exp(omega0 * (firstSegmentTime + endOfDoubleSupport - timeOnEntry));
         double initialProjection = (1.0 - Math.exp(omega0 * (upcomingInitialDoubleSupport - timeOnExit)));

         shouldBe.zero();
         shouldBe.set(0, 0, initialMultiplier * initialProjection);
         shouldBe.set(1, 0, omega0 * initialMultiplier * initialProjection);
         shouldBe.set(2, 0, (1.0 - Math.exp(-omega0 * thirdSegmentTime)));
         shouldBe.set(3, 0, -omega0 * Math.exp(-omega0 * thirdSegmentTime));

         swingExitCMPProjectionMatrix.compute(upcomingDoubleSupportDuration, doubleSupportDuration, singleSupportDuration, omega0, false);

         JUnitTools.assertMatrixEquals(name, shouldBe, swingExitCMPProjectionMatrix, epsilon);



         shouldBe.zero();
         shouldBe.set(2, 0, (1.0 - Math.exp(-omega0 * thirdSegmentTime)));
         shouldBe.set(3, 0, -omega0 * Math.exp(-omega0 * thirdSegmentTime));

         swingExitCMPProjectionMatrix.compute(upcomingDoubleSupportDuration, doubleSupportDuration, singleSupportDuration, omega0, true);

         JUnitTools.assertMatrixEquals(name, shouldBe, swingExitCMPProjectionMatrix, epsilon);
      }
   }
}
