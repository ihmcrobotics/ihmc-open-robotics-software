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

   public static void mult(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      if (a.getNumCols() != b.getNumRows())
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      c.reshape(a.getNumRows(), b.getNumCols());
      nativeCommonOpsWrapper.computeAB(c.data, a.data, b.data, a.getNumRows(), a.getNumCols(), b.getNumCols());
   }

   public static void solve(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F x)
   {
      if (a.getNumRows() != b.getNumRows() || b.getNumCols() != 1 || a.getNumCols() != a.getNumRows())
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      x.reshape(a.getNumCols(), 1);
      nativeCommonOpsWrapper.solve(x.data, a.data, b.data, a.getNumRows());
   }

   public static void solveRobust(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F x)
   {
      if (a.getNumRows() != b.getNumRows() || b.getNumCols() != 1)
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      x.reshape(a.getNumCols(), 1);
      nativeCommonOpsWrapper.solveRobust(x.data, a.data, b.data, a.getNumRows(), a.getNumCols());
   }

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
