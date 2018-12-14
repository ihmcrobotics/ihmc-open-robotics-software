package us.ihmc.robotics.linearAlgebra.commonOps;

public class NativeCommonOpsWrapper
{
   public native void computeAB(double[] result, double[] aData, double[] bData, int aRows, int aCols, int bCols);

   public native void computeAtBA(double[] result, double[] aData, double[] bData, int aRows, int aCols);

   public native void solve(double[] result, double[] aData, double[] bData, int aRows);

   public native void solveRobust(double[] result, double[] aData, double[] bData, int aRows, int aCols);

   public native void solveDamped(double[] result, double[] aData, double[] bData, int aRows, int aCols, double alpha);

   public native void projectOnNullspace(double[] result, double[] aData, double[] bData, int aRows, int aCols, int bRows, double alpha);
}
