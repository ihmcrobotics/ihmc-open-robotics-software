package us.ihmc.commonWalkingControlModules.capturePoint.optimization.qpInput;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationQPSolver;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;

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
    * @param maxMagnitudes the maximum feedback magnitudes in X and Y.
    */
   public void calculateMaxFeedbackMagnitudeConstraint(ICPInequalityInput inputToPack, FrameTuple2DReadOnly maxMagnitudes)
   {
      calculateMaxFeedbackMagnitudeConstraint(inputToPack, maxMagnitudes.getX(), maxMagnitudes.getY());
   }

   /**
    * Computes the inequality constraint for the total feedback (CoP and CMP, if available). This returns a set of 4
    * inequality constraints.
    *
    * @param inputToPack the object in which to store the inequality constraint.
    * @param maxMagnitudes the maximum feedback magnitudes in X and Y.
    */
   public void calculateMaxFeedbackMagnitudeConstraint(ICPInequalityInput inputToPack, DenseMatrix64F maxMagnitudes)
   {
      calculateMaxFeedbackMagnitudeConstraint(inputToPack, maxMagnitudes.get(0), maxMagnitudes.get(1));
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
      inputToPack.reset();
      inputToPack.reshape(4, indexHandler.getNumberOfFreeVariables());

      // set CoP upper bound in the multiplier
      inputToPack.Aineq.set(0, indexHandler.getCoPFeedbackIndex(), 1.0);
      inputToPack.Aineq.set(1, indexHandler.getCoPFeedbackIndex() + 1, 1.0);
      // set CoP lower bound in the multiplier
      inputToPack.Aineq.set(2, indexHandler.getCoPFeedbackIndex(), -1.0);
      inputToPack.Aineq.set(3, indexHandler.getCoPFeedbackIndex() + 1, -1.0);
      if (indexHandler.hasCMPFeedbackTask())
      {
         // add in the CMP effects on the upper bound
         inputToPack.Aineq.set(0, indexHandler.getCMPFeedbackIndex(), 1.0);
         inputToPack.Aineq.set(1, indexHandler.getCMPFeedbackIndex() + 1, 1.0);
         // add in the CMP effects on the lower bound
         inputToPack.Aineq.set(2, indexHandler.getCMPFeedbackIndex(), -1.0);
         inputToPack.Aineq.set(3, indexHandler.getCMPFeedbackIndex() + 1, -1.0);
      }

      inputToPack.bineq.set(0, maxXMagnitude);
      inputToPack.bineq.set(1, maxYMagnitude);
      inputToPack.bineq.set(2, maxXMagnitude);
      inputToPack.bineq.set(3, maxYMagnitude);
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
      double maxYValue = previousYValue + controlDT * maxYRate;

      calculateMaxFeedbackMagnitudeConstraint(inputToPack, maxXValue, maxYValue);
   }
}
