package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationQPSolver;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput.ICPQPInput;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.linearAlgebra.careSolvers.MatrixToolsLocal;

import static us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerQPSolver.cmpFeedbackIndex;
import static us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerQPSolver.copFeedbackIndex;

/**
 * This class is used by the {@link ICPOptimizationQPSolver} to  convert weights and gains into the actual objects for the quadratic program.
 */
public class ICPControllerQPInputCalculator
{
   private final DenseMatrix64F feedbackJacobian = new DenseMatrix64F(2, 6);
   private final DenseMatrix64F feedbackObjective = new DenseMatrix64F(2, 1);
   private final DenseMatrix64F feedbackJtW = new DenseMatrix64F(6, 2);

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(2);

   private final DenseMatrix64F invertedFeedbackGain = new DenseMatrix64F(2, 2);

   /**
    * Computes the CoP feedback minimization task. This simply tries to minimize the total CoP feedback magnitude.
    * Has the form<br>
    *    &delta;<sup>T</sup> Q &delta;<br>
    * where &delta; is the CoP feedback.
    *
    * @param icpQPInputToPack QP input to store the CoP feedback minimization task. Modified.
    * @param feedbackWeight weight attached to minimizing the CoP feedback.
    */
   public static void computeCoPFeedbackMinimizationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F feedbackWeight)
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
   public static void computeCMPFeedbackMinimizationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F cmpFeedbackWeight)
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
   public void computeCoPFeedbackRateMinimizationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F rateWeight, DenseMatrix64F objective)
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
   public void computeCMPFeedbackRateMinimizationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F rateWeight, DenseMatrix64F objective)
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
   public void computeTotalFeedbackRateMinimizationTask(ICPQPInput icpQPInputToPack, DenseMatrix64F rateWeight, DenseMatrix64F objective)
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
                                   DenseMatrix64F currentICPError,
                                   DenseMatrix64F feedbackGain,
                                   DenseMatrix64F weight,
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

      CommonOps.multTransA(feedbackJacobian, weight, feedbackJtW);
      CommonOps.multAdd(feedbackJtW, feedbackJacobian, icpQPInput.quadraticTerm);
      CommonOps.multAdd(feedbackJtW, feedbackObjective, icpQPInput.linearTerm);
      multAddInner(0.5, feedbackObjective, weight, icpQPInput.residualCost);
   }

   public void computeResidualDynamicsError(DenseMatrix64F solution, DenseMatrix64F errorToPack)
   {
      errorToPack.reshape(2, 1);

      CommonOps.mult(feedbackJacobian, solution, errorToPack);

      CommonOps.addEquals(errorToPack, -1.0, feedbackObjective);
      CommonOps.scale(-1.0, errorToPack);
   }

   private void computeQuadraticTask(int startIndex, ICPQPInput icpQPInputToPack, DenseMatrix64F weight, DenseMatrix64F objective)
   {
      computeQuadraticTask(startIndex, icpQPInputToPack, weight, objective, true);
   }

   private void computeQuadraticTask(int startIndex, ICPQPInput icpQPInputToPack, DenseMatrix64F weight, DenseMatrix64F objective, boolean includeResidual)
   {
      MatrixTools.addMatrixBlock(icpQPInputToPack.quadraticTerm, startIndex, startIndex, weight, 0, 0, 2, 2, 1.0);

      MatrixTools.multAddBlock(weight, objective, icpQPInputToPack.linearTerm, startIndex, 0);

      if (includeResidual)
      {
         multAddInner(0.5, objective, weight, icpQPInputToPack.residualCost);
      }
   }

   private final DenseMatrix64F aTb = new DenseMatrix64F(6, 6);

   private void multAddInner(double scalar, DenseMatrix64F jac, DenseMatrix64F weight, DenseMatrix64F resultToPack)
   {
      quadraticMultAddTransA(scalar, jac, weight, jac, resultToPack);
   }

   private void quadraticMultAddTransA(double scalar, DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c, DenseMatrix64F resultToPack)
   {
      aTb.reshape(a.getNumCols(), b.getNumCols());
      CommonOps.multTransA(scalar, a, b, aTb);
      CommonOps.multAdd(aTb, c, resultToPack);
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
                                                        DenseMatrix64F solverInput_H_ToPack,
                                                        DenseMatrix64F solverInput_h_ToPack,
                                                        DenseMatrix64F solverInputResidualCostToPack)
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
                                                        DenseMatrix64F solverInput_H_ToPack,
                                                        DenseMatrix64F solverInput_h_ToPack,
                                                        DenseMatrix64F solverInputResidualCostToPack)
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
                                                         DenseMatrix64F solverInput_H_ToPack,
                                                         DenseMatrix64F solverInput_h_ToPack,
                                                         DenseMatrix64F solverInputResidualCostToPack)
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
                                         DenseMatrix64F solverInput_H_ToPack,
                                         DenseMatrix64F solverInput_h_ToPack,
                                         DenseMatrix64F solverInputResidualCostToPack)
   {
      int size = icpQPInput.linearTerm.getNumRows();
      MatrixTools.addMatrixBlock(solverInput_H_ToPack, 0, 0, icpQPInput.quadraticTerm, 0, 0, size, size, 1.0);
      MatrixTools.addMatrixBlock(solverInput_h_ToPack, 0, 0, icpQPInput.linearTerm, 0, 0, size, 1, 1.0);
      MatrixTools.addMatrixBlock(solverInputResidualCostToPack, 0, 0, icpQPInput.residualCost, 0, 0, 1, 1, 1.0);
   }
}
