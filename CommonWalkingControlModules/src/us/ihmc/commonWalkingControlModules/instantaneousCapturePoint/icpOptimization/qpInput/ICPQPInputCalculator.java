package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import java.util.ArrayList;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class ICPQPInputCalculator
{
   public ICPQPIndexHandler indexHandler;

   public ICPQPInputCalculator(ICPQPIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;
   }

   public void computeFeedbackTask(ICPQPInput icpQPInput, DenseMatrix64F feedbackWeight)
   {
      MatrixTools.addMatrixBlock(icpQPInput.quadraticTerm, 0, 0, feedbackWeight, 0, 0, 2, 2, 1.0);
   }

   private final DenseMatrix64F tmpObjective = new DenseMatrix64F(2, 1);

   public void computeFeedbackRegularizationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F regularizationWeight, DenseMatrix64F objective)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 0, 0, regularizationWeight, 0, 0, 2, 2, 1.0);

      tmpObjective.zero();
      tmpObjective.set(objective);
      CommonOps.mult(regularizationWeight, tmpObjective, tmpObjective);
      CommonOps.multTransA(objective, tmpObjective, icpQPInputToPack.residualCost);

      MatrixTools.addMatrixBlock(icpQPInputToPack.linearTerm, 0, 0, tmpObjective, 0, 0, 2, 1, 1.0);
   }

   public void computeDynamicRelaxationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F dynamicRelaxationWeight)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 0, 0, dynamicRelaxationWeight, 0, 0, 2, 2, 1.0);
   }

   public void computeAngularMomentumMinimizationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F angularMomentumMinimizationWeight)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 0, 0, angularMomentumMinimizationWeight, 0, 0, 2, 2, 1.0);
   }

   public void computeFootstepTask(int footstepNumber, ICPQPInput icpQPInputToPack, DenseMatrix64F footstepWeight, DenseMatrix64F objective)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 2 * footstepNumber, 2 * footstepNumber, footstepWeight, 0, 0, 2, 2, 1.0);

      tmpObjective.zero();
      tmpObjective.set(objective);
      CommonOps.mult(footstepWeight, tmpObjective, tmpObjective);
      CommonOps.multTransA(objective, tmpObjective, icpQPInputToPack.residualCost); // // FIXME: 1/31/17

      MatrixTools.addMatrixBlock(icpQPInputToPack.linearTerm, 2 * footstepNumber, 0, tmpObjective, 0, 0, 2, 1, 1.0);
   }

   public void computeFootstepRegularizationTask(int footstepNumber, ICPQPInput icpQPInputToPack, DenseMatrix64F regularizationWeight, DenseMatrix64F objective)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 2 * footstepNumber, 2 * footstepNumber, regularizationWeight, 0, 0, 2, 2, 1.0);

      tmpObjective.zero();
      tmpObjective.set(objective);
      CommonOps.mult(regularizationWeight, tmpObjective, tmpObjective);
      CommonOps.multTransA(objective, tmpObjective, icpQPInputToPack.residualCost); // // FIXME: 1/31/17

      MatrixTools.addMatrixBlock(icpQPInputToPack.linearTerm, 2 * footstepNumber, 0, tmpObjective, 0, 0, 2, 1, 1.0);
   }

   private static final DenseMatrix64F identity = CommonOps.identity(2, 2);

   public void computeDynamicsConstraint(ICPEqualityConstraintInput icpEqualityInputToPack, DenseMatrix64F currentICP, DenseMatrix64F finalICPRecursion,
         DenseMatrix64F cmpConstantEffect, DenseMatrix64F feedbackGain, ArrayList<DenseMatrix64F> footstepRecursionMultipliers)
   {
      MatrixTools.setMatrixBlock(icpEqualityInputToPack.Aeq, indexHandler.getDynamicRelaxationIndex(), 0, identity, 0, 0, 2, 2, 1.0);

      CommonOps.invert(feedbackGain);
      MatrixTools.setMatrixBlock(icpEqualityInputToPack.Aeq, indexHandler.getFeedbackCMPIndex(), 0, feedbackGain, 0, 0, 2, 2, 1.0);

      if (indexHandler.useStepAdjustment())
      {
         for (int i = 0; i < indexHandler.getNumberOfFootstepsToConsider(); i++)
         {
            MatrixTools.setMatrixBlock(icpEqualityInputToPack.Aeq, 2 * i, 0, footstepRecursionMultipliers.get(i), 0, 0, 2, 2, 1.0);
         }
      }

      CommonOps.subtractEquals(currentICP, finalICPRecursion);
      CommonOps.subtractEquals(currentICP, cmpConstantEffect);

      MatrixTools.setMatrixBlock(icpEqualityInputToPack.beq, 0, 0, currentICP, 0, 0, 2, 1, 1.0);
   }

   public void submitFeedbackTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H, DenseMatrix64F solverInput_h)
   {
      int feedbackCMPIndex = indexHandler.getFeedbackCMPIndex();
      MatrixTools.addMatrixBlock(solverInput_H, feedbackCMPIndex, feedbackCMPIndex, icpQPInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, feedbackCMPIndex, 0, icpQPInput.linearTerm, 0, 0, 2, 1, 1.0);
   }

   public void submitDynamicRelaxationTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H_ToPack, DenseMatrix64F solverInput_h_ToPack)
   {
      int dynamicRelaxationIndex = indexHandler.getDynamicRelaxationIndex();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, dynamicRelaxationIndex, dynamicRelaxationIndex, icpQPInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, dynamicRelaxationIndex, 0, icpQPInput.linearTerm, 0, 0, 2, 1, 1.0);
   }

   public void submitAngularMomentumMinimizationTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H_ToPack, DenseMatrix64F solverInput_h_ToPack)
   {
      int angularMomentumIndex = indexHandler.getAngularMomentumIndex();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, angularMomentumIndex, angularMomentumIndex, icpQPInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, angularMomentumIndex, 0, icpQPInput.linearTerm, 0, 0, 2, 1, 1.0);
   }

   public void submitFootstepTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H, DenseMatrix64F solverInput_h)
   {
      int numberOfFootstepVariables = indexHandler.getNumberOfFootstepVariables();

      int footstepStartIndex = indexHandler.getFootstepStartIndex();
      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, icpQPInput.quadraticTerm, footstepStartIndex, 0, numberOfFootstepVariables, numberOfFootstepVariables, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, 0, 0, icpQPInput.linearTerm, footstepStartIndex, 0, numberOfFootstepVariables, 1, 1.0);
   }
}
