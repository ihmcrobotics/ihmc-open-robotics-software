package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrix;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator;
import us.ihmc.commons.MathTools;

/**
 * This is a coefficient calculator for the portions of the motion function that are not dependent on the contact force magnitudes.
 * That is, they define the CoM position and velocity, whereas the remaining portions of the function contribute to the acceleration.
 * This are the constant and linear coefficients.
 */
public class CoMCoefficientJacobianCalculator
{
   /**
    * Packs the values for the CoM Jacobian into the included matrix. The CoM Jacobian maps from the variable values to the CoM value.
    * These values start at the column specified at the startIndex, which relates the correct variable to the correct segment.
    * @param startIndex index offset for the variables
    * @param time time for the motion function to be computed at
    * @param jacobianToPack matrix that the values are being stored in
    * @param derivative derivative of the motion function to be stored in
    * @param scale scale to apply to the CoM value.
    */
   public static void calculateCoMJacobian(int startIndex, double time, DMatrix jacobianToPack, int derivative, double scale)
   {
      switch (derivative)
      {
         case 0:
            calculatePositionJacobian(startIndex, time, jacobianToPack, scale);
            break;
         case 1:
            calculateVelocityJacobian(startIndex, jacobianToPack, scale);
            break;
         case 2:
            calculateAccelerationJacobian();
            break;
         case 3:
            calculateJerkJacobian();
            break;
         default:
            throw new IllegalArgumentException("Derivative order must be less than 4.");
      }
   }

   /**
    * Packs the values for the DCM Jacobian into the included matrix. The DCM Jacobian maps from the variable values to the DCM value.
    * These values start at the column specified at the startIndex, which relates the correct variable to the correct segment.
    * @param startIndex index offset for the variables
    * @param time time for the motion function to be computed at
    * @param jacobianToPack matrix that the values are being stored in
    * @param derivative derivative of the motion function to be stored in
    * @param scale scale to apply to the DCM value.
    */
   public static void calculateDCMJacobian(int startIndex, double omega, double time, DMatrix jacobianToPack, int derivative, double scale)
   {
      calculateCoMJacobian(startIndex, time, jacobianToPack, derivative, scale);
      calculateCoMJacobian(startIndex, time, jacobianToPack, derivative + 1, scale / omega);
   }

   /**
    * Packs the values for the VRP Jacobian into the included matrix. The VRP Jacobian maps from the variable values to the VRP value.
    * These values start at the column specified at the startIndex, which relates the correct variable to the correct segment.
    * @param startIndex index offset for the variables
    * @param time time for the motion function to be computed at
    * @param jacobianToPack matrix that the values are being stored in
    * @param derivative derivative of the motion function to be stored in
    * @param scale scale to apply to the VRP value.
    */
   public static void calculateVRPJacobian(int startIndex, double omega, double time, DMatrix jacobianToPack, int derivative, double scale)
   {
      calculateCoMJacobian(startIndex, time, jacobianToPack, derivative, scale);
      calculateCoMJacobian(startIndex, time, jacobianToPack, derivative + 2, -scale / (omega * omega));
   }

   private static void calculatePositionJacobian(int startIndex, double time, DMatrix positionJacobianToPack, double scale)
   {
      double c1 = scale;

      add(positionJacobianToPack, 0, startIndex + 1, c1);
      add(positionJacobianToPack, 1, startIndex + 3, c1);
      add(positionJacobianToPack, 2, startIndex + 5, c1);

      if (!MathTools.epsilonEquals(time, 0.0, 1e-5))
      {
         time = Math.min(MPCQPInputCalculator.sufficientlyLongTime, time);
         double c0 = time * c1;
         add(positionJacobianToPack, 0, startIndex, c0);
         add(positionJacobianToPack, 1, startIndex + 2, c0);
         add(positionJacobianToPack, 2, startIndex + 4, c0);
      }
   }

   private static void calculateVelocityJacobian(int startIndex, DMatrix velocityJacobianToPack, double scale)
   {
      add(velocityJacobianToPack, 0, startIndex, scale);
      add(velocityJacobianToPack, 1, startIndex + 2, scale);
      add(velocityJacobianToPack, 2, startIndex + 4, scale);
   }

   private static void calculateAccelerationJacobian()
   {
   }

   private static void calculateJerkJacobian()
   {
   }

   /**
    * Convenience function to allow the use of any dense matrix (e.g. {@link org.ejml.data.DMatrixRMaj} or {@link org.ejml.data.DMatrixSparseCSC}).
    */
   private static void add(DMatrix matrixToPack, int row, int col, double value)
   {
      matrixToPack.set(row, col, value + matrixToPack.unsafe_get(row, col));
   }
}
