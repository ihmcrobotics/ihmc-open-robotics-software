package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

public class FlightS2Function implements S2Segment
{
   private final DMatrixRMaj Afl = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj Bfl = new DMatrixRMaj(6, 3);
   private final DMatrixRMaj g = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj Bflg = new DMatrixRMaj(6, 1);

   private final DMatrixRMaj finalS1 = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj finalS2 = new DMatrixRMaj(6, 1);
   private static final DMatrixRMaj identity = CommonOps_DDRM.identity(3);

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(6, 1);
   private double duration;

   public FlightS2Function(double gravityZ)
   {
      MatrixTools.setMatrixBlock(Afl, 0, 0, identity, 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(Afl, 3, 3, identity, 0, 0, 3, 3, 1.0);

      g.set(2, 0, -Math.abs(gravityZ));
   }

   public void set(DMatrixRMaj finalS1, DMatrixRMaj finalS2, double duration)
   {
      this.finalS1.set(finalS1);
      this.finalS2.set(finalS2);
      this.duration = duration;
   }

   public void compute(double timeInState, DMatrixRMaj s2ToPack)
   {
      double timeRemaining = duration - timeInState;
      MatrixTools.setMatrixBlock(Afl, 0, 3, identity, 0, 0, 3, 3, timeRemaining);

      MatrixTools.setMatrixBlock(Bfl, 0, 0, identity, 0, 0, 3, 3, 0.5 * timeRemaining * timeRemaining);
      MatrixTools.setMatrixBlock(Bfl, 3, 0, identity, 0, 0, 3, 3, timeRemaining);

      CommonOps_DDRM.mult(Bfl, g, Bflg);

      CommonOps_DDRM.mult(2.0, finalS1, Bflg, tempMatrix);
      CommonOps_DDRM.addEquals(tempMatrix, finalS2);

      CommonOps_DDRM.multTransA(Afl, tempMatrix, s2ToPack);
   }
}
