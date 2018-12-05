package us.ihmc.robotics.linearAlgebra.commonOps;

import org.ejml.data.RowD1Matrix64F;

import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class NativeCommonOps
{
   private static final NativeCommonOpsWrapper nativeCommonOpsWrapper = loadNativeOps();

   static NativeCommonOpsWrapper loadNativeOps()
   {
      NativeLibraryLoader.loadLibrary("", "NativeCommonOps");
      return new NativeCommonOpsWrapper();
   }

   /**
    * Computes the matrix multiplication</br>
    * c = a * b
    * @param a matrix in multiplication
    * @param b matrix in multiplication
    * @param c where the result is stored (modified)
    * @throws RuntimeException if the matrix dimensions are incompatible.
    */
   public static void mult(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      if (a.getNumCols() != b.getNumRows())
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      c.reshape(a.getNumRows(), b.getNumCols());
      nativeCommonOpsWrapper.computeAB(c.data, a.data, b.data, a.getNumRows(), a.getNumCols(), b.getNumCols());
   }

   /**
    * Computes the quadratic form</br>
    * c = a' * b * a
    * @param a matrix in multiplication
    * @param b matrix in multiplication
    * @param c where the result is stored (modified)
    * @throws RuntimeException if the matrix dimensions are incompatible.
    */
   public static void multQuad(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      if (a.getNumRows() != b.getNumCols() || b.getNumCols() != b.getNumRows())
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      c.reshape(a.getNumCols(), a.getNumCols());
      nativeCommonOpsWrapper.computeAtBA(c.data, a.data, b.data, a.getNumRows(), a.getNumCols());
   }

   /**
    * Computes the solution to the linear equation</br>
    * a * x == b</br>
    * This method requires that the matrix a is square and invertible and uses a LU decomposition.
    * @param a matrix in equation
    * @param b matrix in equation
    * @param x where the result is stored (modified)
    * @throws RuntimeException if the matrix dimensions are incompatible.
    */
   public static void solve(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F x)
   {
      if (a.getNumRows() != b.getNumRows() || b.getNumCols() != 1 || a.getNumCols() != a.getNumRows())
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      x.reshape(a.getNumCols(), 1);
      nativeCommonOpsWrapper.solve(x.data, a.data, b.data, a.getNumRows());
   }

   /**
    * Computes the solution to the linear equation</br>
    * a * x == b</br>
    * This method does not require that the matrix a is square and invertible and can be used to solve a least square problem. It uses a householder QR decomposition.
    * @param a matrix in equation
    * @param b matrix in equation
    * @param x where the result is stored (modified)
    * @throws RuntimeException if the matrix dimensions are incompatible.
    */
   public static void solveRobust(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F x)
   {
      if (a.getNumRows() != b.getNumRows() || b.getNumCols() != 1)
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      x.reshape(a.getNumCols(), 1);
      nativeCommonOpsWrapper.solveRobust(x.data, a.data, b.data, a.getNumRows(), a.getNumCols());
   }

   /**
    * Computes the solution to the linear equation</br>
    * a * x == b</br>
    * This method uses a damped least square approach and a Cholesky decomposition of</br>
    * a * a' + diag(alpha * alpha)
    * @param a matrix in equation
    * @param b matrix in equation
    * @param alpha damping value
    * @param x where the result is stored (modified)
    * @throws RuntimeException if the matrix dimensions are incompatible.
    */
   public static void solveDamped(RowD1Matrix64F a, RowD1Matrix64F b, double alpha, RowD1Matrix64F x)
   {
      if (a.getNumRows() != b.getNumRows() || b.getNumCols() != 1)
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      x.reshape(a.getNumCols(), 1);
      nativeCommonOpsWrapper.solveDamped(x.data, a.data, b.data, a.getNumRows(), a.getNumCols(), alpha);
   }
}
