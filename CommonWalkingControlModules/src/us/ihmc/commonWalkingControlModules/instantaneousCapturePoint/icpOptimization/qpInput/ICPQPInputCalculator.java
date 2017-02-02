package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;
import org.ejml.data.Matrix;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

import java.util.ArrayList;

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

   public void computeFeedbackRegularizationTask(ICPQPInput icpQPInput, DenseMatrix64F regularizationWeight, DenseMatrix64F objective)
   {
      MatrixTools.addMatrixBlock(icpQPInput.quadraticTerm, 0, 0, regularizationWeight, 0, 0, 2, 2, 1.0);

      tmpObjective.zero();
      tmpObjective.set(objective);
      CommonOps.mult(regularizationWeight, tmpObjective, tmpObjective);
      CommonOps.multTransA(objective, tmpObjective, icpQPInput.residualCost);

      MatrixTools.addMatrixBlock(icpQPInput.linearTerm, 0, 0, tmpObjective, 0, 0, 2, 1, 1.0);
   }

   public void computeDynamicRelaxationTask(ICPQPInput icpQPInput, DenseMatrix64F dynamicRelaxationWeight)
   {
      MatrixTools.addMatrixBlock(icpQPInput.quadraticTerm, 0, 0, dynamicRelaxationWeight, 0, 0, 2, 2, 1.0);
   }

   public void computeFootstepTask(int footstepNumber, ICPQPInput icpQPInput, DenseMatrix64F footstepWeight, DenseMatrix64F objective)
   {
      MatrixTools.addMatrixBlock(icpQPInput.quadraticTerm, 2 * footstepNumber, 2 * footstepNumber, footstepWeight, 0, 0, 2, 2, 1.0);

      tmpObjective.zero();
      tmpObjective.set(objective);
      CommonOps.mult(footstepWeight, tmpObjective, tmpObjective);
      CommonOps.multTransA(objective, tmpObjective, icpQPInput.residualCost); // // FIXME: 1/31/17

      MatrixTools.addMatrixBlock(icpQPInput.linearTerm, 2 * footstepNumber, 0, tmpObjective, 0, 0, 2, 1, 1.0);
   }

   public void computeFootstepRegularizationTask(int footstepNumber, ICPQPInput icpQPInput, DenseMatrix64F regularizationWeight, DenseMatrix64F objective)
   {
      MatrixTools.addMatrixBlock(icpQPInput.quadraticTerm, 2 * footstepNumber, 2 * footstepNumber, regularizationWeight, 0, 0, 2, 2, 1.0);

      tmpObjective.zero();
      tmpObjective.set(objective);
      CommonOps.mult(regularizationWeight, tmpObjective, tmpObjective);
      CommonOps.multTransA(objective, tmpObjective, icpQPInput.residualCost); // // FIXME: 1/31/17 

      MatrixTools.addMatrixBlock(icpQPInput.linearTerm, 2 * footstepNumber, 0, tmpObjective, 0, 0, 2, 1, 1.0);
   }

   private static final DenseMatrix64F identity = CommonOps.identity(2, 2);

   public void computeDynamicsConstraint(ICPEqualityConstraintInput icpEqualityInput, DenseMatrix64F currentICP, DenseMatrix64F finalICPRecursion,
         DenseMatrix64F cmpConstantEffect, DenseMatrix64F feedbackGain, boolean useStepAdjustment, int numberOfSteps, ArrayList<DenseMatrix64F> footstepRecursionMultipliers)
   {
      MatrixTools.setMatrixBlock(icpEqualityInput.Aeq, indexHandler.getDynamicRelaxationIndex(), 0, identity, 0, 0, 2, 2, 1.0);

      CommonOps.invert(feedbackGain);
      MatrixTools.setMatrixBlock(icpEqualityInput.Aeq, indexHandler.getFeedbackCMPIndex(), 0, feedbackGain, 0, 0, 2, 2, 1.0);

      if (useStepAdjustment)
      {
         for (int i = 0; i < numberOfSteps; i++)
         {
            MatrixTools.setMatrixBlock(icpEqualityInput.Aeq, 2 * i, 0, footstepRecursionMultipliers.get(i), 0, 0, 2, 2, 1.0);
         }
      }

      CommonOps.subtractEquals(currentICP, finalICPRecursion);
      CommonOps.subtractEquals(currentICP, cmpConstantEffect);

      MatrixTools.setMatrixBlock(icpEqualityInput.beq, 0, 0, currentICP, 0, 0, 2, 1, 1.0);
   }

   public void submitFeedbackTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H, DenseMatrix64F solverInput_h)
   {
      int feedbackCMPIndex = indexHandler.getFeedbackCMPIndex();
      MatrixTools.addMatrixBlock(solverInput_H, feedbackCMPIndex, feedbackCMPIndex, icpQPInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, feedbackCMPIndex, 0, icpQPInput.linearTerm, 0, 0, 2, 1, 1.0);
   }

   public void submitDynamicRelaxationTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H, DenseMatrix64F solverInput_h)
   {
      int dynamicRelaxationIndex = indexHandler.getDynamicRelaxationIndex();
      MatrixTools.addMatrixBlock(solverInput_H, dynamicRelaxationIndex, dynamicRelaxationIndex, icpQPInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, dynamicRelaxationIndex, 0, icpQPInput.linearTerm, 0, 0, 2, 1, 1.0);
   }

   public void submitFootstepTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H, DenseMatrix64F solverInput_h)
   {
      int numberOfFootstepVariables = indexHandler.getNumberOfFootstepVariables();

      MatrixTools.addMatrixBlock(solverInput_H, 0, 0, icpQPInput.quadraticTerm, 0, 0, numberOfFootstepVariables, numberOfFootstepVariables, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h, 0, 0, icpQPInput.linearTerm, 0, 0, numberOfFootstepVariables, 1, 1.0);
   }
}
