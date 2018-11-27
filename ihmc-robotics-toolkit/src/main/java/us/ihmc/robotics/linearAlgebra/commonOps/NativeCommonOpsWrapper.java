package us.ihmc.robotics.linearAlgebra.commonOps;

public class NativeCommonOpsWrapper
{
   public native void computeAB(double[] result, double[] aData, double[] bData, int aRows, int aCols, int bCols);
}
