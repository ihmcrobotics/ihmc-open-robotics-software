package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.matrixlib.NativeCommonOps;

public class QuadraticS1Function
{
   private final DenseMatrix64F a = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F b = new DenseMatrix64F(6, 6);
   private final DenseMatrix64F combined = new DenseMatrix64F(6, 6);

   private final DenseMatrix64F finalS1 = new DenseMatrix64F(6, 6);

   public void set(DenseMatrix64F linear, DenseMatrix64F constant, DenseMatrix64F finalS1)
   {
      this.finalS1.set(finalS1);
      a.set(linear);
      b.set(constant);
   }

   public void compute(double timeInState, DenseMatrix64F S1ToPack)
   {
      combined.set(b);
      CommonOps.addEquals(combined, timeInState, a);
      NativeCommonOps.multQuad(combined, finalS1, S1ToPack);
   }
}
