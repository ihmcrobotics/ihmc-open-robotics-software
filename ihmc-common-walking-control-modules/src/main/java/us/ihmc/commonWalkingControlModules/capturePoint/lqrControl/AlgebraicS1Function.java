package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.robotics.linearAlgebra.careSolvers.CARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.DefectCorrectionCARESolver;
import us.ihmc.robotics.linearAlgebra.careSolvers.SignFunctionCARESolver;

public class AlgebraicS1Function implements S1Function
{
   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj ARiccati = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj QRiccati = new DMatrixRMaj(6, 6);

   private final CARESolver careSolver = new DefectCorrectionCARESolver(new SignFunctionCARESolver());

   private final DMatrixRMaj S1 = new DMatrixRMaj(6, 6);

   public void set(LQRCommonValues lqrCommonValues)
   {
      set(lqrCommonValues.getQ1(),
          lqrCommonValues.getR1(),
          lqrCommonValues.getR1Inverse(),
          lqrCommonValues.getNTranspose(),
          lqrCommonValues.getA(),
          lqrCommonValues.getB());
   }

   public void set(DMatrixRMaj Q1, DMatrixRMaj R1, DMatrixRMaj R1Inverse, DMatrixRMaj NTranspose, DMatrixRMaj A, DMatrixRMaj B)
   {
      NativeCommonOps.multQuad(NTranspose, R1Inverse, QRiccati);
      CommonOps_DDRM.scale(-1.0, QRiccati);
      CommonOps_DDRM.addEquals(QRiccati, Q1);

      ARiccati.set(A);
      tempMatrix.reshape(3, 6);
      CommonOps_DDRM.mult(R1Inverse, NTranspose, tempMatrix);
      CommonOps_DDRM.multAdd(-1.0, B, tempMatrix, ARiccati);

      careSolver.setMatrices(ARiccati, B, null, QRiccati, R1);
      S1.set(careSolver.computeP());
   }

   public void compute(double timeInState, DMatrixRMaj S1ToPack)
   {
      S1ToPack.set(S1);
   }
}
