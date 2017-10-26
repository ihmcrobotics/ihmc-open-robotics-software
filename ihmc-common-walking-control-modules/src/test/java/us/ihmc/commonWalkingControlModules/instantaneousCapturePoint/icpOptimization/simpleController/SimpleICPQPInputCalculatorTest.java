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
   @Test(timeout = 30000)
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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testAngularMomentumMinimizationTask()
   {
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DenseMatrix64F minimizationWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(minimizationWeight);
      CommonOps.scale(2.0, minimizationWeight);

      CommonOps.setIdentity(icpQPInputExpected.quadraticTerm);
      CommonOps.scale(2.0, icpQPInputExpected.quadraticTerm);

      SimpleICPQPInputCalculator.computeAngularMomentumMinimizationTask(icpQPInputToTest, minimizationWeight);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testFootstepTask()
   {
      SimpleICPQPIndexHandler indexHandler = new SimpleICPQPIndexHandler();
      SimpleICPQPInputCalculator inputCalculator = new SimpleICPQPInputCalculator(indexHandler);

      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(footstepWeight);
      CommonOps.scale(2.0, footstepWeight);

      DenseMatrix64F footstepObjective = new DenseMatrix64F(2, 1);
      footstepObjective.set(0, 0, 0.5);
      footstepObjective.set(0, 0, 0.1);

      icpQPInputExpected.quadraticTerm.set(footstepWeight);

      DenseMatrix64F Qx_p = new DenseMatrix64F(2, 1);
      CommonOps.mult(footstepWeight, footstepObjective, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps.multTransA(footstepObjective, Qx_p, icpQPInputExpected.residualCost);
      CommonOps.scale(0.5, icpQPInputExpected.residualCost);

      inputCalculator.computeFootstepTask(0, icpQPInputToTest, footstepWeight, footstepObjective);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));



      DenseMatrix64F footstepObjective1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective2 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective3 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective4 = new DenseMatrix64F(2, 1);
      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);
      footstepObjective2.set(0, 0, 1.0);
      footstepObjective2.set(1, 0, -0.1);
      footstepObjective3.set(0, 0, 1.5);
      footstepObjective3.set(1, 0, 0.1);
      footstepObjective4.set(0, 0, 2.0);
      footstepObjective4.set(1, 0, -0.1);

      footstepObjective = new DenseMatrix64F(8, 1);
      footstepObjective.set(0, 0, 0.5);
      footstepObjective.set(1, 0, 0.1);
      footstepObjective.set(2, 0, 1.0);
      footstepObjective.set(3, 0, -0.1);
      footstepObjective.set(4, 0, 1.5);
      footstepObjective.set(5, 0, 0.1);
      footstepObjective.set(6, 0, 2.0);
      footstepObjective.set(7, 0, -0.1);

      DenseMatrix64F bigFootstepWeight = new DenseMatrix64F(8, 8);
      CommonOps.setIdentity(bigFootstepWeight);
      CommonOps.scale(2.0, bigFootstepWeight);

      icpQPInputExpected.reshape(8);
      icpQPInputExpected.reset();
      icpQPInputToTest.reshape(8);
      icpQPInputToTest.reset();

      inputCalculator.computeFootstepTask(0, icpQPInputToTest, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepTask(1, icpQPInputToTest, footstepWeight, footstepObjective2);
      inputCalculator.computeFootstepTask(2, icpQPInputToTest, footstepWeight, footstepObjective3);
      inputCalculator.computeFootstepTask(3, icpQPInputToTest, footstepWeight, footstepObjective4);

      icpQPInputExpected.quadraticTerm.set(bigFootstepWeight);

      Qx_p = new DenseMatrix64F(8, 1);
      CommonOps.mult(bigFootstepWeight, footstepObjective, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps.multTransA(footstepObjective, Qx_p, icpQPInputExpected.residualCost);
      CommonOps.scale(0.5, icpQPInputExpected.residualCost);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));
   }
}
