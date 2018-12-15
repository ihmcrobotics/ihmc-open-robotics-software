package us.ihmc.robotics.linearAlgebra.commonOps;

public class NativeCommonOpsWrapper
{
   public native void mult(double[] result, double[] aData, double[] bData, int aRows, int aCols, int bCols);

   public native void multQuad(double[] result, double[] aData, double[] bData, int aRows, int aCols);

   public native void invert(double[] result, double[] aData, int aRows);

   public native void solve(double[] result, double[] aData, double[] bData, int aRows);

   public native boolean solveCheck(double[] result, double[] aData, double[] bData, int aRows);

   public native void solveRobust(double[] result, double[] aData, double[] bData, int aRows, int aCols);

   public native void solveDamped(double[] result, double[] aData, double[] bData, int aRows, int aCols, double alpha);

   public native void projectOnNullspace(double[] result, double[] aData, double[] bData, int aRows, int aCols, int bRows, double alpha);
}
