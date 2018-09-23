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
    * @param maxXMagnitude the maximum feedback magnitude in X.
    * @param maxYMagnitude the maximum feedback magnitude in Y.
    */
   public void calculateMaxFeedbackMagnitudeConstraint(ICPInequalityInput inputToPack, double maxXMagnitude, double maxYMagnitude)
   {
      inputToPack.reset();

      int size = 0;
      boolean hasX = Double.isFinite(maxXMagnitude);
      boolean hasY = Double.isFinite(maxYMagnitude);
      if (hasX)
         size += 2;
      if (hasY)
         size += 2;

      inputToPack.reshape(size, indexHandler.getNumberOfFreeVariables());

      int offset = 0;
      if (hasX)
      { // set X CoP upper and lower bound in the multiplier
         inputToPack.Aineq.set(0, indexHandler.getCoPFeedbackIndex(), 1.0);
         inputToPack.Aineq.set(1, indexHandler.getCoPFeedbackIndex(), -1.0);

         if (indexHandler.hasCMPFeedbackTask())
         { // add in the CMP effects
            inputToPack.Aineq.set(0, indexHandler.getCMPFeedbackIndex(), 1.0);
            inputToPack.Aineq.set(1, indexHandler.getCMPFeedbackIndex(), -1.0);
         }

         inputToPack.bineq.set(0, maxXMagnitude);
         inputToPack.bineq.set(1, maxXMagnitude);

         offset += 2;
      }

      if (hasY)
      { // set X CoP upper and lower bound in the multiplier
         inputToPack.Aineq.set(offset, indexHandler.getCoPFeedbackIndex() + 1, 1.0);
         inputToPack.Aineq.set(offset + 1, indexHandler.getCoPFeedbackIndex() + 1, -1.0);

         if (indexHandler.hasCMPFeedbackTask())
         { // add in the CMP effects
            inputToPack.Aineq.set(offset, indexHandler.getCMPFeedbackIndex() + 1, 1.0);
            inputToPack.Aineq.set(offset + 1, indexHandler.getCMPFeedbackIndex() + 1, -1.0);
         }
         inputToPack.bineq.set(offset, maxYMagnitude);
         inputToPack.bineq.set(offset + 1, maxYMagnitude);
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
   public void calculateMaxFeedbackRateConstraint(ICPInequalityInput inputToPack, FrameTuple2DReadOnly maxRate, FrameTuple2DReadOnly previousValue,
                                                  double controlDT)
   {
      calculateMaxFeedbackRateConstraint(inputToPack, maxRate.getX(), maxRate.getY(), previousValue.getX(), previousValue.getY(), controlDT);
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
   public void calculateMaxFeedbackRateConstraint(ICPInequalityInput inputToPack, double maxRate, FrameTuple2DReadOnly previousValue,
                                                  double controlDT)
   {
      calculateMaxFeedbackRateConstraint(inputToPack, maxRate, maxRate, previousValue.getX(), previousValue.getY(), controlDT);
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
   public void calculateMaxFeedbackRateConstraint(ICPInequalityInput inputToPack, double maxRate, DenseMatrix64F previousValue,
                                                  double controlDT)
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
      double maxYValue = previousYValue + controlDT * maxYRate;

      calculateMaxFeedbackMagnitudeConstraint(inputToPack, maxXValue, maxYValue);
   }
}
