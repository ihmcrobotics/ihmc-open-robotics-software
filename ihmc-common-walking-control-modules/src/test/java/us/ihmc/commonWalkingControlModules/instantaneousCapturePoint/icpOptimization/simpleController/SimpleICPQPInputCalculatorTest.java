package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.jcodec.common.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput.ICPQPInput;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SimpleICPQPInputCalculatorTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testFeedbackTask()
   {
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(2.0, feedbackWeight);

      CommonOps.setIdentity(icpQPInputExpected.quadraticTerm);
      CommonOps.scale(2.0, icpQPInputExpected.quadraticTerm);

      SimpleICPQPInputCalculator.computeFeedbackTask(icpQPInputToTest, feedbackWeight);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testFeedbackRegularizationTask()
   {
      SimpleICPQPIndexHandler indexHandler = new SimpleICPQPIndexHandler();
      SimpleICPQPInputCalculator inputCalculator = new SimpleICPQPInputCalculator(indexHandler);

      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DenseMatrix64F regularizationWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(regularizationWeight);
      CommonOps.scale(2.0, regularizationWeight);

      DenseMatrix64F previousSolution = new DenseMatrix64F(2, 1);
      previousSolution.set(0, 0, 0.5);
      previousSolution.set(0, 0, 0.1);

      icpQPInputExpected.quadraticTerm.set(regularizationWeight);

      DenseMatrix64F Qx_p = new DenseMatrix64F(2, 1);
      CommonOps.mult(regularizationWeight, previousSolution, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps.multTransA(previousSolution, Qx_p, icpQPInputExpected.residualCost);
      CommonOps.scale(0.5, icpQPInputExpected.residualCost);

      inputCalculator.computeFeedbackRegularizationTask(icpQPInputToTest, regularizationWeight, previousSolution);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));
   }
}
