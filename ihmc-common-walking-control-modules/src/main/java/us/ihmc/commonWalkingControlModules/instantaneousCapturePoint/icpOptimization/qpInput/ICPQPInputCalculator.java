package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.qpInput;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController.ICPQPOptimizationSolver;

import java.util.ArrayList;

/**
 * This class is used by the {@link ICPQPOptimizationSolver} to  convert weights and gains into the actual objects for the quadratic program.
 */
public class ICPQPInputCalculator
{
   /** Input calculator that formulates the different objectives and handles adding them to the full program. */
   public ICPQPIndexHandler indexHandler;

   private static final DenseMatrix64F identity = CommonOps.identity(2, 2);
   private final DenseMatrix64F tmpObjective = new DenseMatrix64F(2, 1);

   /**
    * Creates the ICP Quadratic Problem Input Calculator. Refer to the class documentation: {@link ICPQPInputCalculator}.
    *
    * @param indexHandler holder of the indices for the different optimization terms.
    */
   public ICPQPInputCalculator(ICPQPIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;
   }

   /**
    * Computes the CMP feedback minimization task. This simply tries to minimize the total feedback magnitude.
    * Has the form<br>
    *    &delta;<sup>T</sup> Q &delta;<br>
    * where &delta; is the CMP feedback.
    *
    * @param icpQPInputToPack QP input to store the CMP feedback minimization task. Modified.
    * @param feedbackWeight weight attached to minimizing the CMP feedback.
    */
   public void computeFeedbackTask(ICPQPInput icpQPInputToPack, DenseMatrix64F feedbackWeight)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 0, 0, feedbackWeight, 0, 0, 2, 2, 1.0);
   }

   /**
    * Computes the CMP feedback regularization task. This tries to minimize the distance of the current solution
    * from the previous solution. Has the form<br>
    *    (&delta; - &delta;<sub>prev</sub>)<sup>T</sup> Q (&delta; - &delta;<sub>prev</sub>)<br>
    * where &delta; is the CMP feedback and &delta;<sub>prev</sub> is the previous solution.
    *
    * @param icpQPInputToPack QP input to store the CMP feedback regularization task. Modified.
    * @param regularizationWeight weight attached to regularization the CMP feedback.
    * @param objective the previous solution value, &delta;<sub>prev</sub>
    */
   public void computeFeedbackRegularizationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F regularizationWeight, DenseMatrix64F objective)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 0, 0, regularizationWeight, 0, 0, 2, 2, 1.0);

      tmpObjective.zero();
      tmpObjective.set(objective);
      CommonOps.mult(regularizationWeight, tmpObjective, tmpObjective);

      MatrixTools.addMatrixBlock(icpQPInputToPack.linearTerm, 0, 0, tmpObjective, 0, 0, 2, 1, 1.0);

      CommonOps.multTransA(objective, tmpObjective, icpQPInputToPack.residualCost);
      CommonOps.scale(0.5, icpQPInputToPack.residualCost);
   }

   /**
    * Computes the dynamic relaxation minimization task. This simply tries to minimize the dynamic relaxation.
    * Has the form<br>
    *    &epsilon;<sup>T</sup> Q &epsilon;<br>
    * where &epsilon; is the dynamic relaxation.
    *
    * @param icpQPInputToPack QP input to store the dynamic relaxation minimization task. Modified.
    * @param dynamicRelaxationWeight weight attached to minimizing the dynamic relaxation.
    */
   public void computeDynamicRelaxationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F dynamicRelaxationWeight)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 0, 0, dynamicRelaxationWeight, 0, 0, 2, 2, 1.0);
   }

   /**
    * Computes the angular momentum minimization task. This simply tries to minimize the angular momentum.
    * Has the form<br>
    *    &kappa;<sup>T</sup> Q &kappa;<br>
    * where &kappa; is the angular momentum.
    *
    * @param icpQPInputToPack QP input to store the angular momentum minimization task. Modified.
    * @param angularMomentumMinimizationWeight weight attached to minimizing the angular momentum.
    */
   public void computeAngularMomentumMinimizationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F angularMomentumMinimizationWeight)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 0, 0, angularMomentumMinimizationWeight, 0, 0, 2, 2, 1.0);
   }

   /**
    * Computes the step adjustment task for a single footstep. This attempts to have the footstep track a nominal location.
    * Has the form<br>
    *    (r<sub>f</sub> - r<sub>f,r</sub>)<sup>T</sup> Q (r<sub>f</sub> - r<sub>f,r</sub>)<br>
    * where r<sub>f</sub> is the footstep location and r<sub>f,r</sub> is the reference footstep location.
    *
    *
    * @param footstepNumber current footstep number of the task to formulate.
    * @param icpQPInputToPack QP input to store the step adjustment minimization task. Modified.
    * @param footstepWeight weight attached to minimizing the step adjustment.
    * @param objective reference footstep location, r<sub>f,r</sub>
    */
   public void computeFootstepTask(int footstepNumber, ICPQPInput icpQPInputToPack, DenseMatrix64F footstepWeight, DenseMatrix64F objective)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 2 * footstepNumber, 2 * footstepNumber, footstepWeight, 0, 0, 2, 2, 1.0);

      tmpObjective.zero();
      tmpObjective.set(objective);
      CommonOps.mult(footstepWeight, tmpObjective, tmpObjective);

      MatrixTools.addMatrixBlock(icpQPInputToPack.linearTerm, 2 * footstepNumber, 0, tmpObjective, 0, 0, 2, 1, 1.0);

      CommonOps.multTransA(objective, tmpObjective, icpQPInputToPack.residualCost);
      CommonOps.scale(0.5, icpQPInputToPack.residualCost);
   }

   /**
    * Computes the step adjustment regularization task for a single footstep. This attempts to minimize the change from the previous step
    * adjustment solution. Has the form<br>
    *    (r<sub>f</sub> - r<sub>f,prev</sub>)<sup>T</sup> Q (r<sub>f</sub> - r<sub>f,prev</sub>)<br>
    * where r<sub>f</sub> is the footstep location and r<sub>f,r</sub> is the previous footstep location solution.
    *
    *
    * @param footstepNumber current footstep number of the task to formulate.
    * @param icpQPInputToPack QP input to store the step adjustment regularization task. Modified.
    * @param regularizationWeight weight attached to regularizing the step adjustment.
    * @param objective previous footstep location, r<sub>f,prev</sub>
    */
   public void computeFootstepRegularizationTask(int footstepNumber, ICPQPInput icpQPInputToPack, DenseMatrix64F regularizationWeight, DenseMatrix64F objective)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 2 * footstepNumber, 2 * footstepNumber, regularizationWeight, 0, 0, 2, 2, 1.0);

      tmpObjective.zero();
      tmpObjective.set(objective);
      CommonOps.mult(regularizationWeight, tmpObjective, tmpObjective);

      MatrixTools.addMatrixBlock(icpQPInputToPack.linearTerm, 2 * footstepNumber, 0, tmpObjective, 0, 0, 2, 1, 1.0);

      CommonOps.multTransA(objective, tmpObjective, icpQPInputToPack.residualCost);
      CommonOps.scale(0.5, icpQPInputToPack.residualCost);
   }

   /**
    * Computes the recursive dynamics constraint for the ICP Optimization solver. The observers that the reference ICP location is a linear
    * function of the upcoming footstep locations. This defines the relationship between the CMP feedback and the upcoming footstep locations.
    * Has the form<br>
    *    &delta; = k<sub>p</sub> ( &xi; - &Phi;<sub>f</sub> - &Phi;<sub>const</sub> - sum &gamma;<sub>i</sub> r<sub>f,i</sub>) + &epsilon;,<br>
    * where
    * <li>&delta; is the CMP feedback action.</li>
    * <li>k<sub>p</sub> is the ICP proportional feedback gain.</li>
    * <li>&xi; is the current ICP location.</li>
    * <li>&Phi;<sub>f</sub> is the value of the final ICP recursion.</li>
    * <li>&Phi;<sub>const</sub> is the value of the recursive effects of the CMP offsets in the upcoming footsteps and the stance CMP values.</li>
    * <li>&gamma;<sub>i</sub> is the recursion multiplier of the i<sup>th</sup> footstep.</li>
    * <li>r<sub>f,i</sub> is the i<sup>th</sup> footstep.</li>
    * <li>&epsilon; is the dynamic relaxation slack variable that prevents over constraining the problem.</li>
    *
    * @param icpEqualityInputToPack equality constraint for the QP to store the dynamics constraint. Modified.
    * @param currentICP current location of the icp, &xi;.
    * @param finalICPRecursion final ICP recursion value, &Phi;<sub>f</sub>.
    * @param cmpConstantEffect recursive effects of the upcoming CMP offsets and the current stance CMP location, &Phi;<sub>const</sub>.
    * @param feedbackGain proportional ICP feedback gain, k<sub>p</sub>.
    * @param footstepRecursionMultipliers list of upcoming footstep recursion multipliers, &gamma;<sub>i</sub>.
    */
   public void computeDynamicsConstraint(ICPEqualityConstraintInput icpEqualityInputToPack, DenseMatrix64F currentICP, DenseMatrix64F finalICPRecursion,
         DenseMatrix64F cmpConstantEffect, DenseMatrix64F feedbackGain, ArrayList<DenseMatrix64F> footstepRecursionMultipliers)
   {
      MatrixTools.setMatrixBlock(icpEqualityInputToPack.Aeq, 0, indexHandler.getDynamicRelaxationIndex(), identity, 0, 0, 2, 2, 1.0);

      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGain);
      MatrixTools.setMatrixBlock(icpEqualityInputToPack.Aeq, 0, indexHandler.getFeedbackCMPIndex(), feedbackGain, 0, 0, 2, 2, 1.0);

      if (indexHandler.useStepAdjustment())
      {
         for (int i = 0; i < indexHandler.getNumberOfFootstepsToConsider(); i++)
         {
            MatrixTools.setMatrixBlock(icpEqualityInputToPack.Aeq, 0, 2 * i, footstepRecursionMultipliers.get(i), 0, 0, 2, 2, 1.0);
         }
      }

      CommonOps.subtractEquals(currentICP, finalICPRecursion);
      CommonOps.subtractEquals(currentICP, cmpConstantEffect);

      MatrixTools.setMatrixBlock(icpEqualityInputToPack.beq, 0, 0, currentICP, 0, 0, 2, 1, 1.0);
   }

   /**
    * Submits the CMP feedback action task to the total quadratic program cost terms.
    *
    * @param icpQPInput QP Input that stores the data.
    * @param solverInput_H_ToPack full problem quadratic cost term. Modified.
    * @param solverInput_h_ToPack full problem linear cost term. Modified.
    * @param solverInputResidualCost full problem residual cost term.
    */
   public void submitFeedbackTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H_ToPack, DenseMatrix64F solverInput_h_ToPack,
         DenseMatrix64F solverInputResidualCost)
   {
      int feedbackCMPIndex = indexHandler.getFeedbackCMPIndex();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, feedbackCMPIndex, feedbackCMPIndex, icpQPInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, feedbackCMPIndex, 0, icpQPInput.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCost, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
   }

   /**
    * Submits the dynamic relaxation minimization task to the total quadratic program cost terms.
    *
    * @param icpQPInput QP Input that stores the data.
    * @param solverInput_H_ToPack full problem quadratic cost term.
    * @param solverInput_h_ToPack full problem linear cost term.
    * @param solverInputResidualCost full problem residual cost term.
    */
   public void submitDynamicRelaxationTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H_ToPack, DenseMatrix64F solverInput_h_ToPack,
         DenseMatrix64F solverInputResidualCost)
   {
      int dynamicRelaxationIndex = indexHandler.getDynamicRelaxationIndex();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, dynamicRelaxationIndex, dynamicRelaxationIndex, icpQPInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, dynamicRelaxationIndex, 0, icpQPInput.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCost, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
   }

   /**
    * Submits the angular momentum minimization task to the total quadratic program cost terms.
    *
    * @param icpQPInput QP Input that stores the data.
    * @param solverInput_H_ToPack full problem quadratic cost term.
    * @param solverInput_h_ToPack full problem linear cost term.
    * @param solverInputResidualCost full problem residual cost term.
    */
   public void submitAngularMomentumMinimizationTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H_ToPack, DenseMatrix64F solverInput_h_ToPack,
         DenseMatrix64F solverInputResidualCost)
   {
      int angularMomentumIndex = indexHandler.getAngularMomentumIndex();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, angularMomentumIndex, angularMomentumIndex, icpQPInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, angularMomentumIndex, 0, icpQPInput.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCost, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
   }

   /**
    * Submits the footstep adjustment minimization task to the total quadratic program cost terms.
    *
    * @param icpQPInput QP Input that stores the data.
    * @param solverInput_H_ToPack full problem quadratic cost term.
    * @param solverInput_h_ToPack full problem linear cost term.
    * @param solverInputResidualCost full problem residual cost term.
    */
   public void submitFootstepTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H_ToPack, DenseMatrix64F solverInput_h_ToPack,
         DenseMatrix64F solverInputResidualCost)
   {
      int numberOfFootstepVariables = indexHandler.getNumberOfFootstepVariables();

      int footstepStartIndex = indexHandler.getFootstepStartIndex();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, 0, 0, icpQPInput.quadraticTerm, footstepStartIndex, 0, numberOfFootstepVariables, numberOfFootstepVariables, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, 0, 0, icpQPInput.linearTerm, footstepStartIndex, 0, numberOfFootstepVariables, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCost, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
   }
}
