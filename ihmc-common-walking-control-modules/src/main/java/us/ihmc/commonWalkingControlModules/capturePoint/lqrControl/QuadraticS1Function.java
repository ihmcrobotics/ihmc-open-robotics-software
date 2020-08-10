package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.matrixlib.NativeCommonOps;

public class QuadraticS1Function
{
   private final DMatrixRMaj a = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj b = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj combined = new DMatrixRMaj(6, 6);

   private final DMatrixRMaj finalS1 = new DMatrixRMaj(6, 6);

   public void set(DMatrixRMaj linear, DMatrixRMaj constant, DMatrixRMaj finalS1)
   {
      this.finalS1.set(finalS1);
      a.set(linear);
      b.set(constant);
   }

   public void compute(double timeInState, DMatrixRMaj S1ToPack)
   {
      combined.set(b);
      CommonOps_DDRM.addEquals(combined, timeInState, a);
      NativeCommonOps.multQuad(combined, finalS1, S1ToPack);
   }
}
