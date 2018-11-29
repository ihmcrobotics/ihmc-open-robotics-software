package us.ihmc.robotics.linearAlgebra.commonOps;

import org.ejml.data.RowD1Matrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.log.LogTools;
import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class NativeCommonOps
{
   private static final NativeCommonOpsWrapper nativeCommonOpsWrapper = loadNativeOps();
   public static final boolean useNativeOps = nativeCommonOpsWrapper != null;

   static NativeCommonOpsWrapper loadNativeOps()
   {
      try
      {
         NativeLibraryLoader.loadLibrary("", "NativeCommonOps");
         return new NativeCommonOpsWrapper();
      }
      catch (UnsatisfiedLinkError e)
      {
         LogTools.warn("Was not able to load native matrix operations using EJML.");
         return null;
      }
   }

   public static void mult(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      c.reshape(a.getNumRows(), b.getNumCols());
      if (useNativeOps)
      {
         multNative(a, b, c);
      }
      else
      {
         CommonOps.mult(a, b, c);
      }
   }

   public static void solve(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F x)
   {
      x.reshape(a.getNumCols(), 1);
      if (useNativeOps)
      {
         solveNative(a, b, x);
      }
      else
      {
         throw new RuntimeException("Is only supported native.");
      }
   }

   public static void solveRobust(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F x)
   {
      x.reshape(a.getNumCols(), 1);
      if (useNativeOps)
      {
         solveRobustNative(a, b, x);
      }
      else
      {
         throw new RuntimeException("Is only supported native.");
      }
   }

   public static void solveDamped(RowD1Matrix64F a, RowD1Matrix64F b, double alpha, RowD1Matrix64F x)
   {
      x.reshape(a.getNumCols(), 1);
      if (useNativeOps)
      {
         solveDampedNative(a, b, alpha, x);
      }
      else
      {
         throw new RuntimeException("Is only supported native.");
      }
   }

   private static void multNative(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      if (a.getNumCols() != b.getNumRows())
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      nativeCommonOpsWrapper.computeAB(c.data, a.data, b.data, a.getNumRows(), a.getNumCols(), b.getNumCols());
   }

   private static void solveNative(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F x)
   {
      if (a.getNumRows() != b.getNumRows() || b.getNumCols() != 1 || a.getNumCols() != a.getNumRows())
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      nativeCommonOpsWrapper.solve(x.data, a.data, b.data, a.getNumRows());
   }

   private static void solveRobustNative(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F x)
   {
      if (a.getNumRows() != b.getNumRows() || b.getNumCols() != 1)
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      nativeCommonOpsWrapper.solveRobust(x.data, a.data, b.data, a.getNumRows(), a.getNumCols());
   }

   private static void solveDampedNative(RowD1Matrix64F a, RowD1Matrix64F b, double alpha, RowD1Matrix64F x)
   {
      if (a.getNumRows() != b.getNumRows() || b.getNumCols() != 1)
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      nativeCommonOpsWrapper.solveDamped(x.data, a.data, b.data, a.getNumRows(), a.getNumCols(), alpha);
   }
}
