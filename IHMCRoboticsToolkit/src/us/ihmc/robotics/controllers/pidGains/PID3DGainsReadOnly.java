package us.ihmc.robotics.controllers.pidGains;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPID3DGains;

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
   public abstract double[] getProportionalGains();

   /**
    * Returns the derivative PID gains for all three dimensions. The returned
    * array is of length three.
    *
    * @return the derivative PID gains as double array.
    */
   public abstract double[] getDerivativeGains();

   /**
    * Returns the derivative PID gains for all three dimensions. The returned
    * array is of length three.
    *
    * @return the integral PID gains as double array.
    */
   public abstract double[] getIntegralGains();

   /**
    * Returns the maximum integral error allowed by the PID controller
    *
    * @return the maximum integral error.
    */
   public abstract double getMaximumIntegralError();

   /**
    * Returns the maximum error in the derivative input to the controller that
    * should be considered.
    *
    * @return the maximum derivative error.
    */
   public abstract double getMaximumDerivativeError();

   /**
    * Returns the maximum error in the proportional input to the controller that
    * should be considered.
    *
    * @return the maximum proportional error.
    */
   public abstract double getMaximumProportionalError();

   /**
    * Returns the maximum allowed controller output.
    *
    * @return the maximum output.
    */
   public abstract double getMaximumFeedback();

   /**
    * Returns the maximum allowed controller output rate.
    *
    * @return the maximum output rate.
    */
   public abstract double getMaximumFeedbackRate();

   /**
    * Returns the type of gain coupling for the three dimensions.
    * <p>
    * The gain coupling determines whether all axis should use different
    * gains or if some axis (e.g. the X and Y axis for {@link GainCoupling#XY})
    * should use the same controller gains. This is useful when using the
    * YoVariable implementation of this class {@link DefaultYoPID3DGains} since
    * it will cause the creation of different set of tuning variables.
    * </p>
    * @return the coupling of the controller gains for the three axes.
    */
   public abstract GainCoupling getGainCoupling();

   /**
    * Returns whether the PID controller used an I gain or not (if this
    * returns {@code false} the controller will behave as a PD controller).
    * <p>
    * This is especially useful when using the YoVariable implementation of
    * this class {@link DefaultYoPID3DGains} is used since is will avoid
    * creating the YoVariables for tuning the integration.
    * </p>
    * @return whether the gains include integrator gains.
    */
   public abstract boolean isUseIntegrator();

   /**
    * Will pack the proportional gain matrix. The matrix will be a diagonal
    * matrix with the diagonal elements set to the proportional gains.
    *
    * @param proportialGainMatrixToPack the matrix in which the gains are stored. Modified.
    */
   public default void getProportionalGainMatrix(Matrix3D proportialGainMatrixToPack)
   {
      setMatrixDiagonal(getProportionalGains(), proportialGainMatrixToPack);
   }

   /**
    * Will pack the derivative gain matrix. The matrix will be a diagonal
    * matrix with the diagonal elements set to the derivative gains.
    *
    * @param derivativeGainMatrixToPack the matrix in which the gains are stored. Modified.
    */
   public default void getDerivativeGainMatrix(Matrix3D derivativeGainMatrixToPack)
   {
      setMatrixDiagonal(getDerivativeGains(), derivativeGainMatrixToPack);
   }

   /**
    * Will pack the integral gain matrix. The matrix will be a diagonal
    * matrix with the diagonal elements set to the integral gains.
    *
    * @param integralGainMatrixToPack the matrix in which the gains are stored. Modified.
    */
   public default void getIntegralGainMatrix(Matrix3D integralGainMatrixToPack)
   {
      setMatrixDiagonal(getIntegralGains(), integralGainMatrixToPack);
   }

   /**
    * Helper method to fill the gain matrices. Will set the matrix to a diagonal
    * matrix with the diagonal elements equal to the provided array. The array is
    * expected to be of length three.
    *
    * @param diagonalElements the diagonal elements of the matrix
    * @param matrixToFill the matrix that will be set diagonal. Modified.
    */
   static void setMatrixDiagonal(double[] diagonalElements, Matrix3D matrixToFill)
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
}
