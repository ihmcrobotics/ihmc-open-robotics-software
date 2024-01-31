package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerQPSolver.cmpFeedbackIndex;
import static us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerQPSolver.copFeedbackIndex;

import org.ejml.data.DMatrixRMaj;

import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.capturePoint.controller.qpInput.ICPInequalityInput;
import us.ihmc.matrixlib.MatrixTools;

/**
 * This class is used by the {@link ICPOptimizationQPSolver} to calculate variable constraints for the QP Solver
 */
public class ICPControllerQPConstraintCalculator
{

   private static final DMatrixRMaj identity = CommonOps_DDRM.identity(2);

   /**
    * Computes the inequality constraint for the total feedback (CoP and CMP, if available). This returns a set of 4
    * inequality constraints.
    *
    * @param inputToPack the object in which to store the inequality constraint.
    * @param maxXMagnitude the maximum feedback magnitude in X.
    * @param maxYMagnitude the maximum feedback magnitude in Y.
    */
   public static void calculateMaxFeedbackMagnitudeConstraint(ICPInequalityInput inputToPack,
                                                              DMatrixRMaj maxFeedbackTransform,
                                                              double maxXMagnitude,
                                                              double maxYMagnitude,
                                                              boolean useAngularMomentum)
   {
      calculateMaxFeedbackMagnitudeConstraint(inputToPack, maxFeedbackTransform, -maxXMagnitude, maxXMagnitude, -maxYMagnitude, maxYMagnitude, useAngularMomentum);
   }

   public static void calculateMaxFeedbackMagnitudeConstraint(ICPInequalityInput inputToPack,
                                                              double maxXMagnitude,
                                                              double maxYMagnitude,
                                                              boolean useAngularMomentum)
   {
      calculateMaxFeedbackMagnitudeConstraint(inputToPack, identity, -maxXMagnitude, maxXMagnitude, -maxYMagnitude, maxYMagnitude, useAngularMomentum);
   }

   private static void calculateMaxFeedbackMagnitudeConstraint(ICPInequalityInput inputToPack,
                                                              DMatrixRMaj maxFeedbackTransform,
                                                              double minXMagnitude,
                                                              double maxXMagnitude,
                                                              double minYMagnitude,
                                                              double maxYMagnitude,
                                                              boolean useAngularMomentum)
   {
      inputToPack.reset();

      int size = 0;
      boolean hasXMax = Double.isFinite(maxXMagnitude);
      boolean hasXMin = Double.isFinite(minXMagnitude);
      boolean hasYMax = Double.isFinite(maxYMagnitude);
      boolean hasYMin = Double.isFinite(minYMagnitude);

      if (hasXMax || hasYMax)
         size += 2;
      if (hasXMin || hasYMin)
         size += 2;


      int numberOfFreeVariables = useAngularMomentum ? 4 : 2;
      inputToPack.reshape(size, numberOfFreeVariables);

      int offset = 0;
      if (hasXMax || hasYMax)
      { // set X & Y CoP upper bound
         MatrixTools.setMatrixBlock(inputToPack.Aineq, offset, copFeedbackIndex, maxFeedbackTransform, 0, 0, 2, 2, 1.0);

         if (useAngularMomentum)
         { // add in the CMP effects
            MatrixTools.setMatrixBlock(inputToPack.Aineq, offset, cmpFeedbackIndex, maxFeedbackTransform, 0, 0, 2, 2, 1.0);
         }
         inputToPack.bineq.set(offset, maxXMagnitude);
         inputToPack.bineq.set(offset + 1, maxYMagnitude);

         offset += 2;
      }

      if (hasXMin || hasYMin)
      { // set X & Y CoP lower bound
         MatrixTools.setMatrixBlock(inputToPack.Aineq, offset, copFeedbackIndex, maxFeedbackTransform, 0, 0, 2, 2, -1.0);

         if (useAngularMomentum)
         { // add in the CMP effects
            MatrixTools.setMatrixBlock(inputToPack.Aineq, offset, cmpFeedbackIndex, maxFeedbackTransform, 0, 0, 2, 2, -1.0);
         }
         inputToPack.bineq.set(offset, -minXMagnitude);
         inputToPack.bineq.set(offset + 1, -minYMagnitude);

         offset += 2;
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
   public static void calculateMaxFeedbackRateConstraint(ICPInequalityInput inputToPack,
                                                         double maxRate,
                                                         DMatrixRMaj previousValue,
                                                         double controlDT,
                                                         boolean useAngularMomentum)
   {
      calculateMaxFeedbackRateConstraint(inputToPack, maxRate, maxRate, previousValue.get(0), previousValue.get(1), controlDT, useAngularMomentum);
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
   public static void calculateMaxFeedbackRateConstraint(ICPInequalityInput inputToPack,
                                                         double maxXRate,
                                                         double maxYRate,
                                                         double previousXValue,
                                                         double previousYValue,
                                                         double controlDT,
                                                         boolean useAngularMomentum)
   {
      double maxXValue = previousXValue + controlDT * maxXRate;
      double minXValue = previousXValue - controlDT * maxXRate;
      double maxYValue = previousYValue + controlDT * maxYRate;
      double minYValue = previousYValue - controlDT * maxYRate;

      calculateMaxFeedbackMagnitudeConstraint(inputToPack, new DMatrixRMaj(2, 2), minXValue, maxXValue, minYValue, maxYValue, useAngularMomentum);
   }
}
