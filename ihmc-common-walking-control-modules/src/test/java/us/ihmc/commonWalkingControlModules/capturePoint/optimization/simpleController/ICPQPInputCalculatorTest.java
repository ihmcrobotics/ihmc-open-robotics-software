package us.ihmc.commonWalkingControlModules.capturePoint.optimization.simpleController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;
import org.jcodec.common.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput.ICPQPIndexHandler;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput.ICPQPInputCalculator;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput.ICPQPInput;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ICPQPInputCalculatorTest
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

      ICPQPInputCalculator.computeFeedbackTask(icpQPInputToTest, feedbackWeight);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testFeedbackRegularizationTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

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

      ICPQPInputCalculator.computeAngularMomentumMinimizationTask(icpQPInputToTest, minimizationWeight);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testFootstepTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testFootstepRegularizationTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

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

      inputCalculator.computeFootstepRegularizationTask(0, icpQPInputToTest, footstepWeight, footstepObjective);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));


      // test multiple footsteps
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

      inputCalculator.computeFootstepRegularizationTask(0, icpQPInputToTest, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRegularizationTask(1, icpQPInputToTest, footstepWeight, footstepObjective2);
      inputCalculator.computeFootstepRegularizationTask(2, icpQPInputToTest, footstepWeight, footstepObjective3);
      inputCalculator.computeFootstepRegularizationTask(3, icpQPInputToTest, footstepWeight, footstepObjective4);

      icpQPInputExpected.quadraticTerm.set(bigFootstepWeight);

      Qx_p = new DenseMatrix64F(8, 1);
      CommonOps.mult(bigFootstepWeight, footstepObjective, Qx_p);

      icpQPInputExpected.linearTerm.set(Qx_p);

      CommonOps.multTransA(footstepObjective, Qx_p, icpQPInputExpected.residualCost);
      CommonOps.scale(0.5, icpQPInputExpected.residualCost);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testComputeDynamicsTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      // problem requirements
      ICPQPInput icpQPInputToTest = new ICPQPInput(2);
      ICPQPInput icpQPInputExpected = new ICPQPInput(2);

      double omega = 3.0;
      double timeRemainingInState = 1.0;
      double footstepRecursionMultiplier = Math.exp(-omega * timeRemainingInState);

      double gain = 2.5;
      DenseMatrix64F feedbackGain = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGain);
      CommonOps.scale(gain, feedbackGain);

      DenseMatrix64F invertedFeedbackGain = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(invertedFeedbackGain);
      CommonOps.scale(1.0 / gain, invertedFeedbackGain);

      double weight = 4.7;
      DenseMatrix64F weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);


      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      // test just feedback
      indexHandler.setUseAngularMomentum(false);
      indexHandler.computeProblemSize();

      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      icpQPInputExpected.reshape(2);
      icpQPInputToTest.reshape(2);

      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      DenseMatrix64F tmpObjective = new DenseMatrix64F(2, 2);
      CommonOps.mult(weightMatrix, invertedFeedbackGain, tmpObjective);
      CommonOps.mult(invertedFeedbackGain, tmpObjective, icpQPInputExpected.quadraticTerm);

      tmpObjective = new DenseMatrix64F(2, 1);
      CommonOps.mult(weightMatrix, currentICPError, tmpObjective);
      CommonOps.multTransA(invertedFeedbackGain, tmpObjective, icpQPInputExpected.linearTerm);
      CommonOps.multTransA(0.5, currentICPError, tmpObjective, icpQPInputExpected.residualCost);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));

      // check all the permutations
      weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      icpQPInputToTest.reset();
      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));

      icpQPInputToTest.reset();
      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);
      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));

      icpQPInputToTest.reset();
      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);
      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));




      // test feedback and angular momentum
      indexHandler.setUseAngularMomentum(true);
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);

      icpQPInputExpected.reshape(4);
      icpQPInputToTest.reshape(4);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      DenseMatrix64F tmpJacobian = new DenseMatrix64F(2, 4);
      MatrixTools.setMatrixBlock(tmpJacobian, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(tmpJacobian, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);

      DenseMatrix64F JtW = new DenseMatrix64F(4, 2);
      CommonOps.multTransA(tmpJacobian, weightMatrix, JtW);
      CommonOps.mult(JtW, tmpJacobian, icpQPInputExpected.quadraticTerm);

      tmpObjective = new DenseMatrix64F(2, 1);
      CommonOps.mult(weightMatrix, currentICPError, tmpObjective);
      CommonOps.multTransA(tmpJacobian, tmpObjective, icpQPInputExpected.linearTerm);

      DenseMatrix64F OtW = new DenseMatrix64F(1, 2);
      CommonOps.multTransA(currentICPError, weightMatrix, OtW);
      CommonOps.mult(0.5, OtW, currentICPError, icpQPInputExpected.residualCost);

      Assert.assertTrue(MatrixFeatures.isEquals(icpQPInputExpected.quadraticTerm, icpQPInputToTest.quadraticTerm, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(icpQPInputExpected.linearTerm, icpQPInputToTest.linearTerm, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(icpQPInputExpected.residualCost, icpQPInputToTest.residualCost, 1e-7));
      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));

      // check all the permutations
      weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      icpQPInputToTest.reset();
      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));

      icpQPInputToTest.reset();
      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);
      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));

      icpQPInputToTest.reset();
      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);
      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));





      // test just the footstep adjustment independent of feedback
      indexHandler.setUseAngularMomentum(false);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);
      inputCalculator.setConsiderFeedbackInAdjustment(false);

      icpQPInputExpected.reshape(4);
      icpQPInputToTest.reshape(4);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      weightMatrix = new DenseMatrix64F(4, 4);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      tmpJacobian = new DenseMatrix64F(4, 4);
      tmpObjective = new DenseMatrix64F(4, 4);
      tmpJacobian.set(2, 2, footstepRecursionMultiplier);
      tmpJacobian.set(3, 3, footstepRecursionMultiplier);
      MatrixTools.setMatrixBlock(tmpJacobian, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      CommonOps.mult(weightMatrix, tmpJacobian, tmpObjective);
      CommonOps.multTransA(tmpJacobian, tmpObjective, icpQPInputExpected.quadraticTerm);

      tmpObjective = new DenseMatrix64F(4, 1);
      MatrixTools.setMatrixBlock(tmpObjective, 0, 0, currentICPError, 0, 0, 2, 1, 1.0);
      MatrixTools.setMatrixBlock(tmpObjective, 2, 0, referenceFootstepLocation, 0, 0, 2, 1, footstepRecursionMultiplier);
      MatrixTools.addMatrixBlock(tmpObjective, 2, 0, currentICPError, 0, 0, 2, 1, 1.0);

      JtW = new DenseMatrix64F(4, 4);
      CommonOps.multTransA(tmpJacobian, weightMatrix, JtW);
      CommonOps.mult(JtW, tmpObjective, icpQPInputExpected.linearTerm);

      OtW = new DenseMatrix64F(1, 4);

      CommonOps.multTransA(tmpObjective, weightMatrix, OtW);
      CommonOps.mult(0.5, OtW, tmpObjective, icpQPInputExpected.residualCost);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));

      weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      // test the other permutation
      icpQPInputToTest.reset();
      inputCalculator.setConsiderFeedbackInAdjustment(false);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);



      // test just the footstep adjustment combined with feedback
      indexHandler.resetFootsteps();
      indexHandler.setUseAngularMomentum(false);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      icpQPInputExpected.reshape(4);
      icpQPInputToTest.reshape(4);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      tmpJacobian = new DenseMatrix64F(2, 4);
      tmpObjective = new DenseMatrix64F(2, 4);
      MatrixTools.setMatrixBlock(tmpJacobian, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      tmpJacobian.set(0, 2, footstepRecursionMultiplier);
      tmpJacobian.set(1, 3, footstepRecursionMultiplier);
      CommonOps.mult(weightMatrix, tmpJacobian, tmpObjective);
      CommonOps.multTransA(tmpJacobian, tmpObjective, icpQPInputExpected.quadraticTerm);

      tmpObjective = new DenseMatrix64F(2, 1);
      MatrixTools.setMatrixBlock(tmpObjective, 0, 0, currentICPError, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(tmpObjective, 0, 0, referenceFootstepLocation, 0, 0, 2, 1, footstepRecursionMultiplier);

      JtW = new DenseMatrix64F(4, 2);
      CommonOps.multTransA(tmpJacobian, weightMatrix, JtW);
      CommonOps.mult(JtW, tmpObjective, icpQPInputExpected.linearTerm);

      OtW = new DenseMatrix64F(1, 2);

      CommonOps.multTransA(tmpObjective, weightMatrix, OtW);
      CommonOps.mult(0.5, OtW, tmpObjective, icpQPInputExpected.residualCost);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));

      // test the other permutation
      icpQPInputToTest.reset();
      inputCalculator.setConsiderFeedbackInAdjustment(true);
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);




      // test footstep adjustment combined with feedback and angular momentum
      indexHandler.resetFootsteps();
      indexHandler.setUseAngularMomentum(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      icpQPInputExpected.reshape(6);
      icpQPInputToTest.reshape(6);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      tmpJacobian = new DenseMatrix64F(2, 6);
      tmpObjective = new DenseMatrix64F(2, 6);
      MatrixTools.setMatrixBlock(tmpJacobian, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(tmpJacobian, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      tmpJacobian.set(0, 4, footstepRecursionMultiplier);
      tmpJacobian.set(1, 5, footstepRecursionMultiplier);
      CommonOps.mult(weightMatrix, tmpJacobian, tmpObjective);
      CommonOps.multTransA(tmpJacobian, tmpObjective, icpQPInputExpected.quadraticTerm);

      tmpObjective = new DenseMatrix64F(2, 1);
      MatrixTools.setMatrixBlock(tmpObjective, 0, 0, currentICPError, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(tmpObjective, 0, 0, referenceFootstepLocation, 0, 0, 2, 1, footstepRecursionMultiplier);

      JtW = new DenseMatrix64F(6, 2);
      CommonOps.multTransA(tmpJacobian, weightMatrix, JtW);
      CommonOps.mult(JtW, tmpObjective, icpQPInputExpected.linearTerm);

      OtW = new DenseMatrix64F(1, 2);

      CommonOps.multTransA(tmpObjective, weightMatrix, OtW);
      CommonOps.mult(0.5, OtW, tmpObjective, icpQPInputExpected.residualCost);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));




      // test footstep adjustment independent of feedback and angular momentum
      indexHandler.resetFootsteps();
      indexHandler.setUseAngularMomentum(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(false);
      inputCalculator.setConsiderFeedbackInAdjustment(false);

      icpQPInputExpected.reshape(6);
      icpQPInputToTest.reshape(6);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      weightMatrix = new DenseMatrix64F(4, 4);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      tmpJacobian = new DenseMatrix64F(4, 6);
      tmpObjective = new DenseMatrix64F(4, 6);
      MatrixTools.setMatrixBlock(tmpJacobian, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(tmpJacobian, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      tmpJacobian.set(2, 4, footstepRecursionMultiplier);
      tmpJacobian.set(3, 5, footstepRecursionMultiplier);
      CommonOps.mult(weightMatrix, tmpJacobian, tmpObjective);
      CommonOps.multTransA(tmpJacobian, tmpObjective, icpQPInputExpected.quadraticTerm);

      tmpObjective = new DenseMatrix64F(4, 1);
      MatrixTools.setMatrixBlock(tmpObjective, 0, 0, currentICPError, 0, 0, 2, 1, 1.0);
      MatrixTools.setMatrixBlock(tmpObjective, 2, 0, referenceFootstepLocation, 0, 0, 2, 1, footstepRecursionMultiplier);
      MatrixTools.addMatrixBlock(tmpObjective, 2, 0, currentICPError, 0, 0, 2, 1, 1.0);

      JtW = new DenseMatrix64F(6, 4);
      CommonOps.multTransA(tmpJacobian, weightMatrix, JtW);
      CommonOps.mult(JtW, tmpObjective, icpQPInputExpected.linearTerm);

      OtW = new DenseMatrix64F(1, 4);
      CommonOps.multTransA(tmpObjective, weightMatrix, OtW);
      CommonOps.mult(0.5, OtW, tmpObjective, icpQPInputExpected.residualCost);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));




      // test just the footstep adjustment and angular momentum independent of feedback
      indexHandler.resetFootsteps();
      indexHandler.setUseAngularMomentum(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(false);

      icpQPInputExpected.reshape(6);
      icpQPInputToTest.reshape(6);

      icpQPInputExpected.reset();
      icpQPInputToTest.reset();

      weightMatrix = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      inputCalculator.computeDynamicsTask(icpQPInputToTest, currentICPError, referenceFootstepLocation, feedbackGain, weightMatrix, footstepRecursionMultiplier, 1.0);

      weightMatrix = new DenseMatrix64F(4, 4);
      CommonOps.setIdentity(weightMatrix);
      CommonOps.scale(weight, weightMatrix);

      tmpJacobian = new DenseMatrix64F(4, 6);
      tmpObjective = new DenseMatrix64F(4, 6);
      MatrixTools.setMatrixBlock(tmpJacobian, 0, 0, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(tmpJacobian, 0, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(tmpJacobian, 2, 2, invertedFeedbackGain, 0, 0, 2, 2, 1.0);
      tmpJacobian.set(2, 4, footstepRecursionMultiplier);
      tmpJacobian.set(3, 5, footstepRecursionMultiplier);
      CommonOps.mult(weightMatrix, tmpJacobian, tmpObjective);
      CommonOps.multTransA(tmpJacobian, tmpObjective, icpQPInputExpected.quadraticTerm);

      tmpObjective = new DenseMatrix64F(4, 1);
      MatrixTools.setMatrixBlock(tmpObjective, 0, 0, currentICPError, 0, 0, 2, 1, 1.0);
      MatrixTools.setMatrixBlock(tmpObjective, 2, 0, referenceFootstepLocation, 0, 0, 2, 1, footstepRecursionMultiplier);
      MatrixTools.addMatrixBlock(tmpObjective, 2, 0, currentICPError, 0, 0, 2, 1, 1.0);

      JtW = new DenseMatrix64F(6, 4);
      CommonOps.multTransA(tmpJacobian, weightMatrix, JtW);
      CommonOps.mult(JtW, tmpObjective, icpQPInputExpected.linearTerm);

      OtW = new DenseMatrix64F(1, 4);
      CommonOps.multTransA(tmpObjective, weightMatrix, OtW);
      CommonOps.mult(0.5, OtW, tmpObjective, icpQPInputExpected.residualCost);

      Assert.assertTrue(icpQPInputExpected.equals(icpQPInputToTest));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testSubmitFeedbackTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      ICPQPInput feedbackTask = new ICPQPInput(2);
      ICPQPInput feedbackRegularizationTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeFeedbackTask(feedbackTask, feedbackWeight);

      DenseMatrix64F feedbackRegularizationWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackRegularizationWeight);
      CommonOps.scale(1.3, feedbackRegularizationWeight);

      DenseMatrix64F previousFeedbackSolution = new DenseMatrix64F(2, 1);
      previousFeedbackSolution.set(0, 0, 0.03);
      previousFeedbackSolution.set(0, 0, 0.05);

      inputCalculator.computeFeedbackRegularizationTask(feedbackRegularizationTask, feedbackRegularizationWeight, previousFeedbackSolution);

      DenseMatrix64F quadratic = new DenseMatrix64F(2, 2);
      DenseMatrix64F linear = new DenseMatrix64F(2, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(2, 2);
      DenseMatrix64F linearExpected = new DenseMatrix64F(2, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitFeedbackTask(feedbackTask, quadratic, linear, scalar);
      inputCalculator.submitFeedbackTask(feedbackRegularizationTask, quadratic, linear, scalar);

      quadraticExpected.set(feedbackTask.quadraticTerm);
      linearExpected.set(feedbackTask.linearTerm);
      scalarExpected.set(feedbackTask.residualCost);

      CommonOps.addEquals(quadraticExpected, feedbackRegularizationTask.quadraticTerm);
      CommonOps.addEquals(linearExpected, feedbackRegularizationTask.linearTerm);
      CommonOps.addEquals(scalarExpected, feedbackRegularizationTask.residualCost);

      Assert.assertTrue(MatrixFeatures.isEquals(quadraticExpected, quadratic, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(linearExpected, linear, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(scalarExpected, scalar, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testSubmitDynamicsTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setUseAngularMomentum(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F dynamicsWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(dynamicsWeight);
      CommonOps.scale(2.7, dynamicsWeight);

      DenseMatrix64F feedbackGains = new DenseMatrix64F(2, 2);
      DenseMatrix64F invertedFeedbackGains = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGains);
      CommonOps.scale(2.7, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier, safetyFactor);

      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F linear = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F linearExpected = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitDynamicsTask(dynamicsTask, quadratic, linear, scalar);

      quadraticExpected.set(dynamicsTask.quadraticTerm);
      linearExpected.set(dynamicsTask.linearTerm);
      scalarExpected.set(dynamicsTask.residualCost);

      Assert.assertTrue(MatrixFeatures.isEquals(quadraticExpected, quadratic, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(linearExpected, linear, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(scalarExpected, scalar, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testSubmitAngularMomentumMinimizationTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setUseAngularMomentum(true);
      indexHandler.computeProblemSize();

      ICPQPInput angularMomentumTask = new ICPQPInput(2);

      DenseMatrix64F angularMomentumWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(angularMomentumWeight);
      CommonOps.scale(2.0, angularMomentumWeight);

      ICPQPInputCalculator.computeAngularMomentumMinimizationTask(angularMomentumTask, angularMomentumWeight);

      DenseMatrix64F quadratic = new DenseMatrix64F(4, 4);
      DenseMatrix64F linear = new DenseMatrix64F(4, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(4, 4);
      DenseMatrix64F linearExpected = new DenseMatrix64F(4, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitAngularMomentumMinimizationTask(angularMomentumTask, quadratic, linear, scalar);

      MatrixTools.setMatrixBlock(quadraticExpected, 2, 2, angularMomentumTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(linearExpected, 2, 0, angularMomentumTask.linearTerm, 0, 0, 2, 1, 1.0);
      CommonOps.addEquals(scalarExpected, angularMomentumTask.residualCost);

      Assert.assertTrue(MatrixFeatures.isEquals(quadraticExpected, quadratic, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(linearExpected, linear, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(scalarExpected, scalar, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testSubmitFootstepTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();

      ICPQPInput footstepTask = new ICPQPInput(8);
      ICPQPInput footstepRegularizationTask = new ICPQPInput(8);

      DenseMatrix64F footstepObjective1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective2 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective3 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepObjective4 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious2 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious3 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious4 = new DenseMatrix64F(2, 1);

      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);
      footstepObjective2.set(0, 0, 1.0);
      footstepObjective2.set(1, 0, -0.1);
      footstepObjective3.set(0, 0, 1.5);
      footstepObjective3.set(1, 0, 0.1);
      footstepObjective4.set(0, 0, 2.0);
      footstepObjective4.set(1, 0, -0.1);

      footstepPrevious1.set(0, 0, 0.4);
      footstepPrevious1.set(1, 0, 0.09);
      footstepPrevious2.set(0, 0, 1.02);
      footstepPrevious2.set(1, 0, -0.08);
      footstepPrevious3.set(0, 0, 1.51);
      footstepPrevious3.set(1, 0, 0.095);
      footstepPrevious4.set(0, 0, 2.03);
      footstepPrevious4.set(1, 0, -0.14);

      DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
      DenseMatrix64F footstepRegularizationWeight = new DenseMatrix64F(2, 2);

      footstepWeight.set(0, 0, 5.0);
      footstepWeight.set(1, 1, 5.0);
      footstepRegularizationWeight.set(0, 0, 0.1);
      footstepRegularizationWeight.set(1, 1, 0.1);


      inputCalculator.computeFootstepTask(0, footstepTask, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepTask(1, footstepTask, footstepWeight, footstepObjective2);
      inputCalculator.computeFootstepTask(2, footstepTask, footstepWeight, footstepObjective3);
      inputCalculator.computeFootstepTask(3, footstepTask, footstepWeight, footstepObjective4);

      inputCalculator.computeFootstepRegularizationTask(0, footstepRegularizationTask, footstepRegularizationWeight, footstepPrevious1);
      inputCalculator.computeFootstepRegularizationTask(1, footstepRegularizationTask, footstepRegularizationWeight, footstepPrevious2);
      inputCalculator.computeFootstepRegularizationTask(2, footstepRegularizationTask, footstepRegularizationWeight, footstepPrevious3);
      inputCalculator.computeFootstepRegularizationTask(3, footstepRegularizationTask, footstepRegularizationWeight, footstepPrevious4);

      DenseMatrix64F quadratic = new DenseMatrix64F(10, 10);
      DenseMatrix64F linear = new DenseMatrix64F(10, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(10, 10);
      DenseMatrix64F linearExpected = new DenseMatrix64F(10, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitFootstepTask(footstepTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepRegularizationTask, quadratic, linear, scalar);

      MatrixTools.setMatrixBlock(quadraticExpected, 2, 2, footstepTask.quadraticTerm, 0, 0, 8, 8, 1.0);
      MatrixTools.addMatrixBlock(quadraticExpected, 2, 2, footstepRegularizationTask.quadraticTerm, 0, 0, 8, 8, 1.0);
      MatrixTools.setMatrixBlock(linearExpected, 2, 0, footstepTask.linearTerm, 0, 0, 8, 1, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 2, 0, footstepRegularizationTask.linearTerm, 0, 0, 8, 1, 1.0);
      scalarExpected.set(footstepTask.residualCost);
      CommonOps.addEquals(scalarExpected, footstepRegularizationTask.residualCost);

      Assert.assertTrue(MatrixFeatures.isEquals(quadraticExpected, quadratic, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(linearExpected, linear, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(scalarExpected, scalar, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testSubmitFeedbackAndAngularMomentumTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setUseAngularMomentum(true);
      indexHandler.computeProblemSize();

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput angularMomentumTask = new ICPQPInput(2);

      DenseMatrix64F angularMomentumWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(angularMomentumWeight);
      CommonOps.scale(2.0, angularMomentumWeight);

      ICPQPInputCalculator.computeAngularMomentumMinimizationTask(angularMomentumTask, angularMomentumWeight);


      DenseMatrix64F quadratic = new DenseMatrix64F(4, 4);
      DenseMatrix64F linear = new DenseMatrix64F(4, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(4, 4);
      DenseMatrix64F linearExpected = new DenseMatrix64F(4, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitFeedbackTask(feedbackTask, quadratic, linear, scalar);
      inputCalculator.submitAngularMomentumMinimizationTask(angularMomentumTask, quadratic, linear, scalar);

      MatrixTools.setMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.setMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.setMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 2, 2, angularMomentumTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 2, 0, angularMomentumTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, angularMomentumTask.residualCost, 0, 0, 1, 1, 1.0);

      Assert.assertTrue(MatrixFeatures.isEquals(quadraticExpected, quadratic, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(linearExpected, linear, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(scalarExpected, scalar, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testSubmitFeedbackAndDynamicsTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setUseAngularMomentum(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeFeedbackTask(feedbackTask, feedbackWeight);


      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F dynamicsWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(dynamicsWeight);
      CommonOps.scale(2.7, dynamicsWeight);

      DenseMatrix64F feedbackGains = new DenseMatrix64F(2, 2);
      DenseMatrix64F invertedFeedbackGains = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGains);
      CommonOps.scale(1.5, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier, safetyFactor);

      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F linear = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F linearExpected = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitDynamicsTask(dynamicsTask, quadratic, linear, scalar);
      inputCalculator.submitFeedbackTask(feedbackTask, quadratic, linear, scalar);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, dynamicsTask.quadraticTerm, 0, 0, 6, 6, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, dynamicsTask.linearTerm, 0, 0, 6, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, dynamicsTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      Assert.assertTrue(MatrixFeatures.isEquals(quadraticExpected, quadratic, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(linearExpected, linear, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(scalarExpected, scalar, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testSubmitFeedbackAndFootstepTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.registerFootstep();
      indexHandler.setUseAngularMomentum(true);
      indexHandler.computeProblemSize();

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeFeedbackTask(feedbackTask, feedbackWeight);

      ICPQPInput footstepTask = new ICPQPInput(2);
      ICPQPInput footstepRegularizationTask = new ICPQPInput(2);

      DenseMatrix64F footstepObjective1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious1 = new DenseMatrix64F(2, 1);

      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);

      footstepPrevious1.set(0, 0, 0.4);
      footstepPrevious1.set(1, 0, 0.09);

      DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
      DenseMatrix64F footstepRegularizationWeight = new DenseMatrix64F(2, 2);

      footstepWeight.set(0, 0, 5.0);
      footstepWeight.set(1, 1, 5.0);
      footstepRegularizationWeight.set(0, 0, 0.1);
      footstepRegularizationWeight.set(1, 1, 0.1);

      inputCalculator.computeFootstepTask(0, footstepTask, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRegularizationTask(0, footstepRegularizationTask, footstepRegularizationWeight, footstepPrevious1);

      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F linear = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F linearExpected = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitFootstepTask(footstepTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepRegularizationTask, quadratic, linear, scalar);
      inputCalculator.submitFeedbackTask(feedbackTask, quadratic, linear, scalar);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepRegularizationTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepRegularizationTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepRegularizationTask.residualCost, 0, 0, 1, 1, 1.0);


      Assert.assertTrue(MatrixFeatures.isEquals(quadraticExpected, quadratic, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(linearExpected, linear, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(scalarExpected, scalar, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testSubmitFeedbackAndFootstepAndDynamicsTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setUseAngularMomentum(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeFeedbackTask(feedbackTask, feedbackWeight);


      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F dynamicsWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(dynamicsWeight);
      CommonOps.scale(2.7, dynamicsWeight);

      DenseMatrix64F feedbackGains = new DenseMatrix64F(2, 2);
      DenseMatrix64F invertedFeedbackGains = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGains);
      CommonOps.scale(1.5, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier, safetyFactor);

      ICPQPInput footstepTask = new ICPQPInput(2);
      ICPQPInput footstepRegularizationTask = new ICPQPInput(2);

      DenseMatrix64F footstepObjective1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious1 = new DenseMatrix64F(2, 1);

      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);

      footstepPrevious1.set(0, 0, 0.4);
      footstepPrevious1.set(1, 0, 0.09);

      DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
      DenseMatrix64F footstepRegularizationWeight = new DenseMatrix64F(2, 2);

      footstepWeight.set(0, 0, 5.0);
      footstepWeight.set(1, 1, 5.0);
      footstepRegularizationWeight.set(0, 0, 0.1);
      footstepRegularizationWeight.set(1, 1, 0.1);

      inputCalculator.computeFootstepTask(0, footstepTask, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRegularizationTask(0, footstepRegularizationTask, footstepRegularizationWeight, footstepPrevious1);
      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F linear = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F linearExpected = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitDynamicsTask(dynamicsTask, quadratic, linear, scalar);
      inputCalculator.submitFeedbackTask(feedbackTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepRegularizationTask, quadratic, linear, scalar);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, dynamicsTask.quadraticTerm, 0, 0, 6, 6, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, dynamicsTask.linearTerm, 0, 0, 6, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, dynamicsTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepRegularizationTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepRegularizationTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepRegularizationTask.residualCost, 0, 0, 1, 1, 1.0);

      Assert.assertTrue(MatrixFeatures.isEquals(quadraticExpected, quadratic, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(linearExpected, linear, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(scalarExpected, scalar, 1e-7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 21000)
   public void testSubmitFeedbackAndFootstepAndDynamicsAndAngularMomentumTask()
   {
      ICPQPIndexHandler indexHandler = new ICPQPIndexHandler();
      ICPQPInputCalculator inputCalculator = new ICPQPInputCalculator(indexHandler);

      indexHandler.setUseAngularMomentum(true);
      indexHandler.registerFootstep();
      indexHandler.computeProblemSize();
      inputCalculator.setConsiderAngularMomentumInAdjustment(true);
      inputCalculator.setConsiderFeedbackInAdjustment(true);

      ICPQPInput feedbackTask = new ICPQPInput(2);

      DenseMatrix64F feedbackWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackWeight);
      CommonOps.scale(3.6, feedbackWeight);

      ICPQPInputCalculator.computeFeedbackTask(feedbackTask, feedbackWeight);


      ICPQPInput dynamicsTask = new ICPQPInput(6);

      double footstepRecursionMultiplier = Math.exp(-3.0 * 1.2);
      double safetyFactor = 1.0;

      DenseMatrix64F currentICPError = new DenseMatrix64F(2, 1);
      currentICPError.set(0, 0, 0.03);
      currentICPError.set(1, 0, 0.06);

      DenseMatrix64F referenceFootstepLocation = new DenseMatrix64F(2, 1);
      referenceFootstepLocation.set(0, 0, 0.5);
      referenceFootstepLocation.set(1, 0, 0.1);

      DenseMatrix64F dynamicsWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(dynamicsWeight);
      CommonOps.scale(2.7, dynamicsWeight);

      DenseMatrix64F feedbackGains = new DenseMatrix64F(2, 2);
      DenseMatrix64F invertedFeedbackGains = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(feedbackGains);
      CommonOps.scale(1.5, feedbackGains);
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGains, invertedFeedbackGains);

      inputCalculator.computeDynamicsTask(dynamicsTask, currentICPError, referenceFootstepLocation, feedbackGains, dynamicsWeight, footstepRecursionMultiplier, safetyFactor);

      ICPQPInput footstepTask = new ICPQPInput(2);
      ICPQPInput footstepRegularizationTask = new ICPQPInput(2);

      DenseMatrix64F footstepObjective1 = new DenseMatrix64F(2, 1);
      DenseMatrix64F footstepPrevious1 = new DenseMatrix64F(2, 1);

      footstepObjective1.set(0, 0, 0.5);
      footstepObjective1.set(1, 0, 0.1);

      footstepPrevious1.set(0, 0, 0.4);
      footstepPrevious1.set(1, 0, 0.09);

      DenseMatrix64F footstepWeight = new DenseMatrix64F(2, 2);
      DenseMatrix64F footstepRegularizationWeight = new DenseMatrix64F(2, 2);

      footstepWeight.set(0, 0, 5.0);
      footstepWeight.set(1, 1, 5.0);
      footstepRegularizationWeight.set(0, 0, 0.1);
      footstepRegularizationWeight.set(1, 1, 0.1);

      inputCalculator.computeFootstepTask(0, footstepTask, footstepWeight, footstepObjective1);
      inputCalculator.computeFootstepRegularizationTask(0, footstepRegularizationTask, footstepRegularizationWeight, footstepPrevious1);

      ICPQPInput angularMomentumTask = new ICPQPInput(2);

      DenseMatrix64F angularMomentumWeight = new DenseMatrix64F(2, 2);
      CommonOps.setIdentity(angularMomentumWeight);
      CommonOps.scale(2.0, angularMomentumWeight);

      ICPQPInputCalculator.computeAngularMomentumMinimizationTask(angularMomentumTask, angularMomentumWeight);



      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F linear = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalar = new DenseMatrix64F(1, 1);

      DenseMatrix64F quadraticExpected = new DenseMatrix64F(6, 6);
      DenseMatrix64F linearExpected = new DenseMatrix64F(6, 1);
      DenseMatrix64F scalarExpected = new DenseMatrix64F(1, 1);

      inputCalculator.submitDynamicsTask(dynamicsTask, quadratic, linear, scalar);
      inputCalculator.submitFeedbackTask(feedbackTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepTask, quadratic, linear, scalar);
      inputCalculator.submitFootstepTask(footstepRegularizationTask, quadratic, linear, scalar);
      inputCalculator.submitAngularMomentumMinimizationTask(angularMomentumTask, quadratic, linear, scalar);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, dynamicsTask.quadraticTerm, 0, 0, 6, 6, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, dynamicsTask.linearTerm, 0, 0, 6, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, dynamicsTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 0, 0, feedbackTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 0, 0, feedbackTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, feedbackTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 4, 4, footstepRegularizationTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 4, 0, footstepRegularizationTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, footstepRegularizationTask.residualCost, 0, 0, 1, 1, 1.0);

      MatrixTools.addMatrixBlock(quadraticExpected, 2, 2, angularMomentumTask.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(linearExpected, 2, 0, angularMomentumTask.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(scalarExpected, 0, 0, angularMomentumTask.residualCost, 0, 0, 1, 1, 1.0);

      Assert.assertTrue(MatrixFeatures.isEquals(quadraticExpected, quadratic, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(linearExpected, linear, 1e-7));
      Assert.assertTrue(MatrixFeatures.isEquals(scalarExpected, scalar, 1e-7));
   }





}
