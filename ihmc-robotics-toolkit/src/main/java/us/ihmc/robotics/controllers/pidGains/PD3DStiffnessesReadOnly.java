package us.ihmc.robotics.controllers.pidGains;

import us.ihmc.euclid.matrix.Matrix3D;

import java.util.Arrays;

/**
 * Read-only interface for PD stiffnesses in 3D. The stiffnesses are defined by
 * the proportional and derivative for each dimension.
 */
public interface PD3DStiffnessesReadOnly
{
   /**
    * Returns the proportional PD stiffnesses for all three dimensions. The returned
    * array is of length three.
    *
    * @return the proportional PD stiffnesses as double array.
    */
   public abstract double[] getProportionalStiffnesses();

   /**
    * Returns the derivative PD stiffnesses for all three dimensions. The returned
    * array is of length three.
    *
    * @return the derivative PD stiffnesses as double array.
    */
   public abstract double[] getDerivativeStiffnesses();

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
    * Will pack the proportional stiffness matrix. The matrix will be a diagonal
    * matrix with the diagonal elements set to the proportional stiffnesss.
    *
    * @param proportialStiffnessMatrixToPack the matrix in which the stiffnesss are stored. Modified.
    */
   public default void getProportionalStiffnessMatrix(Matrix3D proportialStiffnessMatrixToPack)
   {
      setMatrixDiagonal(getProportionalStiffnesses(), proportialStiffnessMatrixToPack);
   }

   /**
    * Will pack the derivative stiffness matrix. The matrix will be a diagonal
    * matrix with the diagonal elements set to the derivative stiffnesss.
    * (Derivative stiffness is also known as damping)
    *
    * @param derivativeStiffnessMatrixToPack the matrix in which the stiffnesss are stored. Modified.
    */
   public default void getDerivativeStiffnessMatrix(Matrix3D derivativeStiffnessMatrixToPack)
   {
      setMatrixDiagonal(getDerivativeStiffnesses(), derivativeStiffnessMatrixToPack);
   }


   /**
    * Helper method to fill the stiffness matrices. Will set the matrix to a diagonal
    * matrix with the diagonal elements equal to the provided array. The array is
    * expected to be of length three.
    *
    * @param diagonalElements the diagonal elements of the matrix
    * @param matrixToFill the matrix that will be set diagonal. Modified.
    */
   static void setMatrixDiagonal(double[] diagonalElements, Matrix3D matrixToFill)
   {
      PD3DStiffnessesReadOnly.checkArrayLength(diagonalElements);
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

   public default boolean equals(PD3DStiffnessesReadOnly other)
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
         if (!Arrays.equals(getProportionalStiffnesses(), other.getProportionalStiffnesses()))
            return false;
         if (!Arrays.equals(getDerivativeStiffnesses(), other.getDerivativeStiffnesses()))
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
