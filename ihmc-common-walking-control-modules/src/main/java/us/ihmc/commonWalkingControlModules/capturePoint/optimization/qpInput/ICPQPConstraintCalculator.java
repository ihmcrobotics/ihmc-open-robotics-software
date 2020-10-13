package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationQPSolver;

/**
 * This class is used by the {@link ICPOptimizationQPSolver} to calculate variable constraints for the QP Solver
 */
public class ICPQPConstraintCalculator
{
   /** Input calculator that formulates the different objectives and handles adding them to the full program. */
   private final ICPQPIndexHandler indexHandler;

   /**
    * Creates the ICP Quadratic Problem Constraint Calculator. Refer to the class documentation: {@link ICPQPConstraintCalculator}.
    *
    * @param indexHandler holder of the indices for the different optimization terms.
    */
   public ICPQPConstraintCalculator(ICPQPIndexHandler indexHandler)
   {
      this.indexHandler = indexHandler;
   }

   /**
    * Computes the inequality constraint for the total feedback (CoP and CMP, if available). This returns a set of 4
    * inequality constraints.
    *
    * @param inputToPack the object in which to store the inequality constraint.
    * @param maxXMagnitude the maximum feedback magnitude in X.
    * @param maxYMagnitude the maximum feedback magnitude in Y.
    */
   public void calculateMaxFeedbackMagnitudeConstraint(ICPInequalityInput inputToPack, double maxXMagnitude, double maxYMagnitude)
   {
      calculateMaxFeedbackMagnitudeConstraint(inputToPack, -maxXMagnitude, maxXMagnitude, -maxYMagnitude, maxYMagnitude);
   }

   public void calculateMaxFeedbackMagnitudeConstraint(ICPInequalityInput inputToPack, double minXMagnitude, double maxXMagnitude, double minYMagnitude,
                                                       double maxYMagnitude)
   {
      inputToPack.reset();

      int size = 0;
      boolean hasXMax = Double.isFinite(maxXMagnitude);
      boolean hasXMin = Double.isFinite(minXMagnitude);
      boolean hasYMax = Double.isFinite(maxYMagnitude);
      boolean hasYMin = Double.isFinite(minYMagnitude);
      if (hasXMax)
         size += 1;
      if (hasXMin)
         size += 1;
      if (hasYMax)
         size += 1;
      if (hasYMin)
         size += 1;

      inputToPack.reshape(size, indexHandler.getNumberOfFreeVariables());

      int offset = 0;
      if (hasXMax)
      { // set X CoP upper bound
         inputToPack.Aineq.set(offset, indexHandler.getCoPFeedbackIndex(), 1.0);

         if (indexHandler.hasCMPFeedbackTask())
         { // add in the CMP effects
            inputToPack.Aineq.set(offset, indexHandler.getCMPFeedbackIndex(), 1.0);
         }
         inputToPack.bineq.set(offset, maxXMagnitude);

         offset += 1;
      }

      if (hasXMin)
      { // set X CoP lower bound
         inputToPack.Aineq.set(offset, indexHandler.getCoPFeedbackIndex(), -1.0);

         if (indexHandler.hasCMPFeedbackTask())
         { // add in the CMP effects
            inputToPack.Aineq.set(offset, indexHandler.getCMPFeedbackIndex(), -1.0);
         }
         inputToPack.bineq.set(offset, -minXMagnitude);

         offset += 1;
      }

      if (hasYMax)
      { // set X CoP upper and lower bound in the multiplier
         inputToPack.Aineq.set(offset, indexHandler.getCoPFeedbackIndex() + 1, 1.0);

         if (indexHandler.hasCMPFeedbackTask())
         { // add in the CMP effects
            inputToPack.Aineq.set(offset, indexHandler.getCMPFeedbackIndex() + 1, 1.0);
         }
         inputToPack.bineq.set(offset, maxYMagnitude);

         offset += 1;
      }

      if (hasYMin)
      { // set X CoP upper and lower bound in the multiplier
         inputToPack.Aineq.set(offset, indexHandler.getCoPFeedbackIndex() + 1, -1.0);

         if (indexHandler.hasCMPFeedbackTask())
         { // add in the CMP effects
            inputToPack.Aineq.set(offset, indexHandler.getCMPFeedbackIndex() + 1, -1.0);
         }
         inputToPack.bineq.set(offset, -minYMagnitude);
      }
   }

   /**
    * Computes the inequality constraint for the to limit the total feedback rate (CoP and CMP, if available). This returns a set of 4
    * inequality constraints.
    *
    * @param inputToPack the object in which to store the inequality constraint.
    * @param maxRate the maximum feedback rate in X and Y.
    * @param previousValue the value of the previous feedback term in X and Y.
    * @param controlDT the time delta at which this solver is run. Should be the control loop DT.
    */
   public void calculateMaxFeedbackRateConstraint(ICPInequalityInput inputToPack, double maxRate, DMatrixRMaj previousValue, double controlDT)
   {
      calculateMaxFeedbackRateConstraint(inputToPack, maxRate, maxRate, previousValue.get(0), previousValue.get(1), controlDT);
   }

   /**
    * Computes the inequality constraint for the to limit the total feedback rate (CoP and CMP, if available). This returns a set of 4
    * inequality constraints.
    *
    * @param inputToPack the object in which to store the inequality constraint.
    * @param maxXRate the maximum feedback rate in X.
    * @param maxYRate the maximum feedback rate in Y.
    * @param previousXValue the value of the previous feedback term in X.
    * @param previousYValue the value of the previous feedback term in Y.
    * @param controlDT the time delta at which this solver is run. Should be the control loop DT.
    */
   public void calculateMaxFeedbackRateConstraint(ICPInequalityInput inputToPack, double maxXRate, double maxYRate, double previousXValue,
                                                  double previousYValue, double controlDT)
   {
      double maxXValue = previousXValue + controlDT * maxXRate;
      double minXValue = previousXValue - controlDT * maxXRate;
      double maxYValue = previousYValue + controlDT * maxYRate;
      double minYValue = previousYValue - controlDT * maxYRate;

      calculateMaxFeedbackMagnitudeConstraint(inputToPack, minXValue, maxXValue, minYValue, maxYValue);
   }
}
