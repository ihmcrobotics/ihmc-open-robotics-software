package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationQPSolver;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

/**
 * This class is used by the {@link ICPOptimizationQPSolver} to  convert weights and gains into the actual objects for the quadratic program.
 */
public class ICPQPInputCalculator
{
   private boolean considerAngularMomentumInAdjustment = true;
   private boolean considerFeedbackInAdjustment = true;

   /** Input calculator that formulates the different objectives and handles adding them to the full program. */
   public ICPQPIndexHandler indexHandler;

   private final DenseMatrix64F identity = CommonOps.identity(2, 2);
   private final DenseMatrix64F tmpObjective = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F tmpScalar = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F feedbackJacobian = new DenseMatrix64F(2, 6);
   private final DenseMatrix64F feedbackJtW = new DenseMatrix64F(6, 2);
   private final DenseMatrix64F feedbackObjective = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F feedbackObjtW = new DenseMatrix64F(1, 2);

   private final DenseMatrix64F adjustmentJacobian = new DenseMatrix64F(2,6);
   private final DenseMatrix64F adjustmentJtW = new DenseMatrix64F(6,2);
   private final DenseMatrix64F adjustmentObjective = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F adjustmentObjtW = new DenseMatrix64F(1, 2);

   private final DenseMatrix64F invertedFeedbackGain = new DenseMatrix64F(2, 2);


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
    * Sets whether or not the footstep adjustment should have knowledge of trying to use angular momentum for balance.
    * By default, this is true.
    */
   public void setConsiderAngularMomentumInAdjustment(boolean considerAngularMomentumInAdjustment)
   {
      this.considerAngularMomentumInAdjustment = considerAngularMomentumInAdjustment;
   }

   /**
    * Sets whether or not the footstep adjustment should have knowledge of trying to use cop feedback for balance.
    * By default, this is true.
    */
   public void setConsiderFeedbackInAdjustment(boolean considerFeedbackInAdjustment)
   {
      this.considerFeedbackInAdjustment = considerFeedbackInAdjustment;
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
   public static void computeFeedbackTask(ICPQPInput icpQPInputToPack, DenseMatrix64F feedbackWeight)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 0, 0, feedbackWeight, 0, 0, 2, 2, 1.0);
   }

   /**
    * Computes the CMP feedback rate task. This tries to minimize the distance of the current solution
    * from the previous solution. Has the form<br>
    *    (&delta; - &delta;<sub>prev</sub>)<sup>T</sup> Q (&delta; - &delta;<sub>prev</sub>)<br>
    * where &delta; is the CMP feedback and &delta;<sub>prev</sub> is the previous solution.
    *
    * @param icpQPInputToPack QP input to store the CMP feedback rate task. Modified.
    * @param rateWeight weight attached to rate the CMP feedback.
    * @param objective the previous solution value, &delta;<sub>prev</sub>
    */
   public void computeFeedbackRateTask(ICPQPInput icpQPInputToPack, DenseMatrix64F rateWeight, DenseMatrix64F objective)
   {
      computeQuadraticTask(0, icpQPInputToPack, rateWeight, objective);
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
   public static void computeAngularMomentumMinimizationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F angularMomentumMinimizationWeight)
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
      int footstepIndex = 2 * footstepNumber;
      computeQuadraticTask(footstepIndex, icpQPInputToPack, footstepWeight, objective);
   }

   /**
    * Computes the step adjustment rate task for a single footstep. This attempts to minimize the change from the previous step
    * adjustment solution. Has the form<br>
    *    (r<sub>f</sub> - r<sub>f,prev</sub>)<sup>T</sup> Q (r<sub>f</sub> - r<sub>f,prev</sub>)<br>
    * where r<sub>f</sub> is the footstep location and r<sub>f,r</sub> is the previous footstep location solution.
    *
    *
    * @param footstepNumber current footstep number of the task to formulate.
    * @param icpQPInputToPack QP input to store the step adjustment rate task. Modified.
    * @param rateWeight weight attached to regularizing the step adjustment.
    * @param objective previous footstep location, r<sub>f,prev</sub>
    */
   public void computeFootstepRateTask(int footstepNumber, ICPQPInput icpQPInputToPack, DenseMatrix64F rateWeight, DenseMatrix64F objective)
   {
      int footstepIndex = 2 * footstepNumber;
      computeQuadraticTask(footstepIndex, icpQPInputToPack, rateWeight, objective);
   }

   private void computeQuadraticTask(int startIndex, ICPQPInput icpQPInputToPack, DenseMatrix64F weight, DenseMatrix64F objective)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, startIndex, startIndex, weight, 0, 0, 2, 2, 1.0);

      CommonOps.mult(weight, objective, tmpObjective);

      MatrixTools.addMatrixBlock(icpQPInputToPack.linearTerm, startIndex, 0, tmpObjective, 0, 0, 2, 1, 1.0);

      CommonOps.multTransA(0.5, objective, tmpObjective, tmpScalar);
      CommonOps.addEquals(icpQPInputToPack.residualCost, tmpScalar);
   }

   public void computeDynamicsTask(ICPQPInput icpQPInput, DenseMatrix64F currentICPError, DenseMatrix64F referenceFootstepLocation,
         DenseMatrix64F feedbackGain, DenseMatrix64F weight, double footstepRecursionMultiplier, double footstepAdjustmentSafetyFactor)
   {
      invertedFeedbackGain.zero();
      DiagonalMatrixTools.invertDiagonalMatrix(feedbackGain, invertedFeedbackGain);

      int size = 2;
      if (indexHandler.useAngularMomentum())
         size += 2;
      if (indexHandler.useStepAdjustment())
         size += 2;

      feedbackJacobian.reshape(2, size);
      feedbackJtW.reshape(size, 2);
      adjustmentJacobian.reshape(2, size);
      adjustmentJtW.reshape(size, 2);

      feedbackJacobian.zero();
      feedbackJtW.zero();
      feedbackObjective.zero();
      feedbackObjtW.zero();

      adjustmentJacobian.zero();
      adjustmentJtW.zero();
      adjustmentObjective.zero();
      adjustmentObjtW.zero();

      MatrixTools.setMatrixBlock(feedbackJacobian, 0, indexHandler.getFeedbackCMPIndex(), invertedFeedbackGain, 0, 0, 2, 2, 1.0);

      if (indexHandler.useAngularMomentum())
         MatrixTools.setMatrixBlock(feedbackJacobian, 0, indexHandler.getAngularMomentumIndex(), invertedFeedbackGain, 0, 0, 2, 2, 1.0);

      if (indexHandler.useStepAdjustment())
      {
         CommonOps.setIdentity(identity);
         CommonOps.scale(footstepRecursionMultiplier / footstepAdjustmentSafetyFactor, identity, identity);

         if (considerFeedbackInAdjustment && (considerAngularMomentumInAdjustment || !indexHandler.useAngularMomentum()))
         { // we only have one task, which is to use feedback and step adjustment
            MatrixTools.setMatrixBlock(feedbackJacobian, 0, indexHandler.getFootstepStartIndex(), identity, 0, 0, 2, 2, 1.0);

            MatrixTools.addMatrixBlock(feedbackObjective, 0, 0, referenceFootstepLocation, 0, 0, 2, 1, footstepRecursionMultiplier);
         }
         else
         {
            MatrixTools.setMatrixBlock(adjustmentJacobian, 0, indexHandler.getFootstepStartIndex(), identity, 0, 0, 2, 2, 1.0);

            if (considerAngularMomentumInAdjustment && indexHandler.useAngularMomentum())
               MatrixTools.setMatrixBlock(adjustmentJacobian, 0, indexHandler.getAngularMomentumIndex(), invertedFeedbackGain, 0, 0, 2, 2, 1.0);

            MatrixTools.addMatrixBlock(adjustmentObjective, 0, 0, referenceFootstepLocation, 0, 0, 2, 1, footstepRecursionMultiplier);
            MatrixTools.addMatrixBlock(adjustmentObjective, 0, 0, currentICPError, 0, 0, 2, 1, 1.0);
         }
      }

      MatrixTools.addMatrixBlock(feedbackObjective, 0, 0, currentICPError, 0, 0, 2, 1, 1.0);

      CommonOps.multTransA(feedbackJacobian, weight, feedbackJtW);
      CommonOps.multTransA(adjustmentJacobian, weight, adjustmentJtW);

      CommonOps.multAdd(feedbackJtW, feedbackJacobian, icpQPInput.quadraticTerm);
      CommonOps.multAdd(adjustmentJtW, adjustmentJacobian, icpQPInput.quadraticTerm);

      CommonOps.multAdd(feedbackJtW, feedbackObjective, icpQPInput.linearTerm);
      CommonOps.multAdd(adjustmentJtW, adjustmentObjective, icpQPInput.linearTerm);


      CommonOps.multTransA(feedbackObjective, weight, feedbackObjtW);
      CommonOps.multTransA(adjustmentObjective, weight, adjustmentObjtW);

      CommonOps.multAdd(0.5, feedbackObjtW, feedbackObjective, icpQPInput.residualCost);
      CommonOps.multAdd(0.5, adjustmentObjtW, adjustmentObjective, icpQPInput.residualCost);
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
   public void submitDynamicsTask(ICPQPInput icpQPInput, DenseMatrix64F solverInput_H_ToPack, DenseMatrix64F solverInput_h_ToPack,
         DenseMatrix64F solverInputResidualCostToPack)
   {
      int size = icpQPInput.linearTerm.getNumRows();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, 0, 0, icpQPInput.quadraticTerm, 0, 0, size, size, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, 0, 0, icpQPInput.linearTerm, 0, 0, size, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCostToPack, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
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
         DenseMatrix64F solverInputResidualCostToPack)
   {
      int angularMomentumIndex = indexHandler.getAngularMomentumIndex();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, angularMomentumIndex, angularMomentumIndex, icpQPInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, angularMomentumIndex, 0, icpQPInput.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCostToPack, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
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
         DenseMatrix64F solverInputResidualCostToPack)
   {
      int numberOfFootstepVariables = indexHandler.getNumberOfFootstepVariables();

      int footstepStartIndex = indexHandler.getFootstepStartIndex();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, footstepStartIndex, footstepStartIndex, icpQPInput.quadraticTerm, 0, 0, numberOfFootstepVariables, numberOfFootstepVariables, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, footstepStartIndex, 0, icpQPInput.linearTerm, 0, 0, numberOfFootstepVariables, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCostToPack, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
   }
}
