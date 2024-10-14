package us.ihmc.wholeBodyControlCore.pidGains;

import java.util.Arrays;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.yoVariables.providers.DoubleProvider;

/**
 * Read-only interface for PID gains in three dimensions.
 */
public interface PID3DGainsReadOnly
{
   /**
    * Returns the proportional PID gains for all three dimensions. The returned
    * array is of length three.
    *
    * @return the proportional PID gains as double array.
    */
   double[] getProportionalGains();

   /**
    * Returns the derivative PID gains for all three dimensions. The returned
    * array is of length three.
    *
    * @return the derivative PID gains as double array.
    */
   double[] getDerivativeGains();

   /**
    * Returns the derivative PID gains for all three dimensions. The returned
    * array is of length three.
    *
    * @return the integral PID gains as double array.
    */
   double[] getIntegralGains();

   /**
    * Returns the maximum integral error allowed by the PID controller
    *
    * @return the maximum integral error.
    */
   double getMaximumIntegralError();

   /**
    * Returns the maximum error in the derivative input to the controller that
    * should be considered.
    *
    * @return the maximum derivative error.
    */
   double getMaximumDerivativeError();

   /**
    * Returns the maximum error in the proportional input to the controller that
    * should be considered.
    *
    * @return the maximum proportional error.
    */
   double getMaximumProportionalError();

   /**
    * Returns the maximum allowed controller output.
    *
    * @return the maximum output.
    */
   double getMaximumFeedback();

   /**
    * Returns the maximum allowed controller output rate.
    *
    * @return the maximum output rate.
    */
   double getMaximumFeedbackRate();

   /**
    * Will pack the proportional gain matrix. The matrix will be a diagonal
    * matrix with the diagonal elements set to the proportional gains.
    *
    * @param proportialGainMatrixToPack the matrix in which the gains are stored. Modified.
    */
   default void getProportionalGainMatrix(Matrix3DBasics proportialGainMatrixToPack)
   {
      packMatrixDiagonal(getProportionalGains(), proportialGainMatrixToPack);
   }

   /**
    * Will pack the derivative gain matrix. The matrix will be a diagonal
    * matrix with the diagonal elements set to the derivative gains.
    *
    * @param derivativeGainMatrixToPack the matrix in which the gains are stored. Modified.
    */
   default void getDerivativeGainMatrix(Matrix3DBasics derivativeGainMatrixToPack)
   {
      packMatrixDiagonal(getDerivativeGains(), derivativeGainMatrixToPack);
   }

   /**
    * Will pack the integral gain matrix. The matrix will be a diagonal
    * matrix with the diagonal elements set to the integral gains.
    *
    * @param integralGainMatrixToPack the matrix in which the gains are stored. Modified.
    */
   default void getIntegralGainMatrix(Matrix3DBasics integralGainMatrixToPack)
   {
      packMatrixDiagonal(getIntegralGains(), integralGainMatrixToPack);
   }

   /**
    * Returns the maximum allowed controller output as a DoubleProvider.
    *
    * @return the maximum output.
    */
   DoubleProvider getMaximumFeedbackProvider();

   /**
    * Returns the maximum allowed controller output rate as a DoubleProvider.
    *
    * @return the maximum output rate.
    */
   DoubleProvider getMaximumFeedbackRateProvider();

   /**
    * Helper method to fill the gain matrices. Will set the matrix to a diagonal
    * matrix with the diagonal elements equal to the provided array. The array is
    * expected to be of length three.
    *
    * @param diagonalElements the diagonal elements of the matrix
    * @param matrixToFill the matrix that will be set diagonal. Modified.
    */
   static void packMatrixDiagonal(double[] diagonalElements, Matrix3DBasics matrixToFill)
   {
      PID3DGainsReadOnly.checkArrayLength(diagonalElements);
      matrixToFill.fill(0.0);
      for (int i = 0; i < 3; i++)
      {
         matrixToFill.setElement(i, i, diagonalElements[i]);
      }
   }

   /**
    * Helper method to check if an array is of expected length.
    *
    * @param array to be checked.
    * @throws RuntimeException if the array is null or not of length three.
    */
   static void checkArrayLength(double[] array)
   {
      if (array == null || array.length != 3)
      {
         throw new RuntimeException("Expected array of length three.");
      }
   }

   default boolean equals(PID3DGainsReadOnly other)
   {
      if (other == null)
      {
         return false;
      }
      else if (other == this)
      {
         return true;
      }
      else
      {
         if (!Arrays.equals(getProportionalGains(), other.getProportionalGains()))
            return false;
         if (!Arrays.equals(getDerivativeGains(), other.getDerivativeGains()))
            return false;
         if (!Arrays.equals(getIntegralGains(), other.getIntegralGains()))
            return false;
         if (Double.compare(getMaximumIntegralError(), other.getMaximumIntegralError()) != 0)
            return false;
         if (Double.compare(getMaximumDerivativeError(), other.getMaximumDerivativeError()) != 0)
            return false;
         if (Double.compare(getMaximumProportionalError(), other.getMaximumProportionalError()) != 0)
            return false;
         if (Double.compare(getMaximumFeedback(), other.getMaximumFeedback()) != 0)
            return false;
         if (Double.compare(getMaximumFeedbackRate(), other.getMaximumFeedbackRate()) != 0)
            return false;

         return true;
      }
   }
}
