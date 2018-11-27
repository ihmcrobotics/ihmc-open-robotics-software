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

   private static void multNative(RowD1Matrix64F a, RowD1Matrix64F b, RowD1Matrix64F c)
   {
      if (a.getNumCols() != b.getNumRows())
      {
         throw new RuntimeException("Incompatible Matrix Dimensions.");
      }
      nativeCommonOpsWrapper.computeAB(c.data, a.data, b.data, a.getNumRows(), a.getNumCols(), b.getNumCols());
   }
}
