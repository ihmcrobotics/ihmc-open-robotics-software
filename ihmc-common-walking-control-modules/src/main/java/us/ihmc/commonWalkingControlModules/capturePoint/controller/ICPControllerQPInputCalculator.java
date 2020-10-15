package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerQPSolver.cmpFeedbackIndex;
import static us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerQPSolver.copFeedbackIndex;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationQPSolver;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput.ICPQPInput;
import us.ihmc.matrixlib.MatrixTools;

/**
 * This class is used by the {@link ICPOptimizationQPSolver} to  convert weights and gains into the actual objects for the quadratic program.
 */
public class ICPControllerQPInputCalculator
{
   private final DMatrixRMaj feedbackJacobian = new DMatrixRMaj(2, 6);
   private final DMatrixRMaj feedbackObjective = new DMatrixRMaj(2, 1);
   private final DMatrixRMaj feedbackJtW = new DMatrixRMaj(6, 2);

   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.linear(2);

   private final DMatrixRMaj invertedFeedbackGain = new DMatrixRMaj(2, 2);

   /**
    * Computes the CoP feedback minimization task. This simply tries to minimize the total CoP feedback magnitude.
    * Has the form<br>
    *    &delta;<sup>T</sup> Q &delta;<br>
    * where &delta; is the CoP feedback.
    *
    * @param icpQPInputToPack QP input to store the CoP feedback minimization task. Modified.
    * @param feedbackWeight weight attached to minimizing the CoP feedback.
    */
   public static void computeCoPFeedbackMinimizationTask(ICPQPInput icpQPInputToPack, DMatrixRMaj feedbackWeight)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 0, 0, feedbackWeight, 0, 0, 2, 2, 1.0);
   }

   /**
    * Computes the angular momentum rate minimization task in the form of the CoP-CMP difference objective.
    * This simply tries to achieve the desired CoP-CMP difference.
    * Has the form<br>
    *    &kappa;<sup>T</sup> Q &kappa;<br>
    * where &kappa; is the difference of the CMP from the CoP.
    *
    * @param icpQPInputToPack QP input to store the angular momentum minimization task. Modified.
    * @param cmpFeedbackWeight weight attached to minimizing the angular momentum rate.
    */
   public static void computeCMPFeedbackMinimizationTask(ICPQPInput icpQPInputToPack, DMatrixRMaj cmpFeedbackWeight)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, 0, 0, cmpFeedbackWeight, 0, 0, 2, 2, 1.0);
   }

   /**
    * Computes the CoP feedback rate minimization task. This tries to minimize the distance of the current solution
    * from the previous solution. Has the form<br>
    *    (&delta; - &delta;<sub>prev</sub>)<sup>T</sup> Q (&delta; - &delta;<sub>prev</sub>)<br>
    * where &delta; is the CoP feedback and &delta;<sub>prev</sub> is the previous solution.
    *
    * @param icpQPInputToPack QP input to store the CoP feedback rate task. Modified.
    * @param rateWeight weight attached to rate the CoP feedback.
    * @param objective the previous solution value, &delta;<sub>prev</sub>
    */
   public void computeCoPFeedbackRateMinimizationTask(ICPQPInput icpQPInputToPack, DMatrixRMaj rateWeight, DMatrixRMaj objective)
   {
      computeQuadraticTask(copFeedbackIndex, icpQPInputToPack, rateWeight, objective);
   }

   /**
    * Computes the CMP feedback rate task. This tries to minimize the distance of the current solution
    * from the previous solution. Has the form<br>
    *    (&kappa; - &kappa;<sub>prev</sub>)<sup>T</sup> Q (&kappa; - &kappa;<sub>prev</sub>)<br>
    * where &kappa; is the CMP feedback and &kappa;<sub>prev</sub> is the previous solution.
    *
    * @param icpQPInputToPack QP input to store the CoP feedback rate task. Modified.
    * @param rateWeight weight attached to rate the CoP feedback.
    * @param objective the previous solution value, &delta;<sub>prev</sub>
    */
   public void computeCMPFeedbackRateMinimizationTask(ICPQPInput icpQPInputToPack, DMatrixRMaj rateWeight, DMatrixRMaj objective)
   {
      computeQuadraticTask(cmpFeedbackIndex, icpQPInputToPack, rateWeight, objective);
   }

   /**
    * Computes the total feedback rate task. This tries to minimize the distance of the current solution
    * from the previous solution. Has the form<br>
    *    (&delta; + &kappa; - &delta;<sub>prev</sub> - &kappa;<sub>prev</sub>)<sup>T</sup> Q (&delta; + &kappa; - &delta;<sub>prev</sub> - &kappa;<sub>prev</sub>)<br>
    * where &delta; is the CoP feedback, &kappa; is the CMP feedback, &delta;<sub>prev</sub> is the previous solution for the CoP feedback,
    * and &kappa;<sub>prev</sub> is the previous solution for the CMP feedback.
    *
    * @param icpQPInputToPack QP input to store the CoP feedback rate task. Modified.
    * @param rateWeight weight attached to rate the CoP feedback.
    * @param objective the previous solution value, &delta;<sub>prev</sub>
    */
   public void computeTotalFeedbackRateMinimizationTask(ICPQPInput icpQPInputToPack, DMatrixRMaj rateWeight, DMatrixRMaj objective)
   {
      computeQuadraticTask(copFeedbackIndex, icpQPInputToPack, rateWeight, objective);
      computeQuadraticTask(cmpFeedbackIndex, icpQPInputToPack, rateWeight, objective, false);
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, copFeedbackIndex, cmpFeedbackIndex, rateWeight, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, cmpFeedbackIndex, copFeedbackIndex, rateWeight, 0, 0, 2, 2, 1.0);
   }

   /**
    * Computes the task to enforce the feedback dynamics in the controller
    */
   public void computeDynamicsTask(ICPQPInput icpQPInput,
                                   DMatrixRMaj currentICPError,
                                   DMatrixRMaj feedbackGain,
                                   DMatrixRMaj weight,
                                   boolean useAngularMomentum)
   {
      invertedFeedbackGain.zero();
      solver.setA(feedbackGain);
      solver.invert(invertedFeedbackGain);

      int size = 2;
      if (useAngularMomentum)
         size += 2;

      feedbackJacobian.reshape(2, size);
      feedbackJtW.reshape(size, 2);

      feedbackJacobian.zero();
      feedbackJtW.zero();
      feedbackObjective.zero();

      MatrixTools.setMatrixBlock(feedbackJacobian, 0, copFeedbackIndex, invertedFeedbackGain, 0, 0, 2, 2, 1.0);

      if (useAngularMomentum)
         MatrixTools.setMatrixBlock(feedbackJacobian, 0, cmpFeedbackIndex, invertedFeedbackGain, 0, 0, 2, 2, 1.0);

      MatrixTools.setMatrixBlock(feedbackObjective, 0, 0, currentICPError, 0, 0, 2, 1, 1.0);

      CommonOps_DDRM.multTransA(feedbackJacobian, weight, feedbackJtW);
      CommonOps_DDRM.multAdd(feedbackJtW, feedbackJacobian, icpQPInput.quadraticTerm);
      CommonOps_DDRM.multAdd(feedbackJtW, feedbackObjective, icpQPInput.linearTerm);
      multAddInner(0.5, feedbackObjective, weight, icpQPInput.residualCost);
   }

   public void computeResidualDynamicsError(DMatrixRMaj solution, DMatrixRMaj errorToPack)
   {
      errorToPack.reshape(2, 1);

      CommonOps_DDRM.mult(feedbackJacobian, solution, errorToPack);

      CommonOps_DDRM.addEquals(errorToPack, -1.0, feedbackObjective);
      CommonOps_DDRM.scale(-1.0, errorToPack);
   }

   private void computeQuadraticTask(int startIndex, ICPQPInput icpQPInputToPack, DMatrixRMaj weight, DMatrixRMaj objective)
   {
      computeQuadraticTask(startIndex, icpQPInputToPack, weight, objective, true);
   }

   private void computeQuadraticTask(int startIndex, ICPQPInput icpQPInputToPack, DMatrixRMaj weight, DMatrixRMaj objective, boolean includeResidual)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, startIndex, startIndex, weight, 0, 0, 2, 2, 1.0);

      MatrixTools.multAddBlock(weight, objective, icpQPInputToPack.linearTerm, startIndex, 0);

      if (includeResidual)
      {
         multAddInner(0.5, objective, weight, icpQPInputToPack.residualCost);
      }
   }

   private final DMatrixRMaj aTb = new DMatrixRMaj(6, 6);

   private void multAddInner(double scalar, DMatrixRMaj jac, DMatrixRMaj weight, DMatrixRMaj resultToPack)
   {
      quadraticMultAddTransA(scalar, jac, weight, jac, resultToPack);
   }

   private void quadraticMultAddTransA(double scalar, DMatrixRMaj a, DMatrixRMaj b, DMatrixRMaj c, DMatrixRMaj resultToPack)
   {
      aTb.reshape(a.getNumCols(), b.getNumCols());
      CommonOps_DDRM.multTransA(scalar, a, b, aTb);
      CommonOps_DDRM.multAdd(aTb, c, resultToPack);
   }

   /**
    * Submits the CoP feedback action task to the total quadratic program cost terms.
    *
    * @param icpQPInput QP Input that stores the data.
    * @param solverInput_H_ToPack full problem quadratic cost term. Modified.
    * @param solverInput_h_ToPack full problem linear cost term. Modified.
    * @param solverInputResidualCostToPack full problem residual cost term.
    */
   public static void submitCoPFeedbackMinimizationTask(ICPQPInput icpQPInput,
                                                        DMatrixRMaj solverInput_H_ToPack,
                                                        DMatrixRMaj solverInput_h_ToPack,
                                                        DMatrixRMaj solverInputResidualCostToPack)
   {
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, copFeedbackIndex, copFeedbackIndex, icpQPInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, copFeedbackIndex, 0, icpQPInput.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCostToPack, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
   }

   /**
    * Submits the CMP feedback action task to the total quadratic program cost terms.
    *
    * @param icpQPInput QP Input that stores the data.
    * @param solverInput_H_ToPack full problem quadratic cost term.
    * @param solverInput_h_ToPack full problem linear cost term.
    * @param solverInputResidualCostToPack full problem residual cost term.
    */
   public static void submitCMPFeedbackMinimizationTask(ICPQPInput icpQPInput,
                                                        DMatrixRMaj solverInput_H_ToPack,
                                                        DMatrixRMaj solverInput_h_ToPack,
                                                        DMatrixRMaj solverInputResidualCostToPack)
   {
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, cmpFeedbackIndex, cmpFeedbackIndex, icpQPInput.quadraticTerm, 0, 0, 2, 2, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, cmpFeedbackIndex, 0, icpQPInput.linearTerm, 0, 0, 2, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCostToPack, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
   }

   /**
    * Submits the CoP feedback action task to the total quadratic program cost terms.
    *
    * @param icpQPInput QP Input that stores the data.
    * @param solverInput_H_ToPack full problem quadratic cost term. Modified.
    * @param solverInput_h_ToPack full problem linear cost term. Modified.
    * @param solverInputResidualCostToPack full problem residual cost term.
    */
   public static void submitFeedbackRateMinimizationTask(ICPQPInput icpQPInput,
                                                         DMatrixRMaj solverInput_H_ToPack,
                                                         DMatrixRMaj solverInput_h_ToPack,
                                                         DMatrixRMaj solverInputResidualCostToPack)
   {
      int size = icpQPInput.linearTerm.getNumRows();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, copFeedbackIndex, copFeedbackIndex, icpQPInput.quadraticTerm, 0, 0, size, size, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, copFeedbackIndex, 0, icpQPInput.linearTerm, 0, 0, size, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCostToPack, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
   }

   /**
    * Submits the dynamic relaxation minimization task to the total quadratic program cost terms.
    *
    * @param icpQPInput QP Input that stores the data.
    * @param solverInput_H_ToPack full problem quadratic cost term.
    * @param solverInput_h_ToPack full problem linear cost term.
    * @param solverInputResidualCostToPack full problem residual cost term.
    */
   public static void submitDynamicsTask(ICPQPInput icpQPInput,
                                         DMatrixRMaj solverInput_H_ToPack,
                                         DMatrixRMaj solverInput_h_ToPack,
                                         DMatrixRMaj solverInputResidualCostToPack)
   {
      int size = icpQPInput.linearTerm.getNumRows();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, 0, 0, icpQPInput.quadraticTerm, 0, 0, size, size, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, 0, 0, icpQPInput.linearTerm, 0, 0, size, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCostToPack, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
   }
}
