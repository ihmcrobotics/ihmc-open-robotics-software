package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;

public class LQRTools
{
   public static void matrixExponential(DenseMatrix64F a, DenseMatrix64F b, int termsForExpansion)
   {
      DenseMatrix64F interiorGuy = CommonOps.identity(a.numRows);
      int denominator = 1;
      CommonOps.setIdentity(b);
      for (int k = 1; k < termsForExpansion; k++)
      {
         denominator *= k;
         CommonOps.mult(a, interiorGuy, interiorGuy);
         CommonOps.addEquals(b, 1.0 / denominator, interiorGuy);
      }
   }
}
