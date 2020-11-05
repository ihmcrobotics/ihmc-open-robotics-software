package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

public class LQRCommonValues
{
   private final DMatrixRMaj A = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj B = new DMatrixRMaj(6, 3);
   private final DMatrixRMaj BTranspose = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj C = new DMatrixRMaj(3, 6);
   private final DMatrixRMaj D = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj Q = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj R = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj Q1 = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj R1 = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj R1Inverse = new DMatrixRMaj(3, 3);

   private final DMatrixRMaj N = new DMatrixRMaj(6, 3);
   private final DMatrixRMaj NTranspose = new DMatrixRMaj(3, 6);

   private final DMatrixRMaj DQ = new DMatrixRMaj(3, 3);


   private final DMatrixRMaj A2 = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj A2Inverse = new DMatrixRMaj(6, 6);
   private final DMatrixRMaj B2 = new DMatrixRMaj(6, 3);

   private final DMatrixRMaj A2InverseB2 = new DMatrixRMaj(6, 3);

   private final DMatrixRMaj Nb = new DMatrixRMaj(3, 6);

   private final DMatrixRMaj R1InverseDQ = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj R1InverseBTranspose = new DMatrixRMaj(3, 6);

   public void computeDynamicsMatrix(double omega)
   {
      MatrixTools.setMatrixBlock(A, 0, 3, CommonOps_DDRM.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(B, 3, 0, CommonOps_DDRM.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(C, 0, 0, CommonOps_DDRM.identity(3, 3), 0, 0, 3, 3, 1.0);
      MatrixTools.setMatrixBlock(D, 0, 0, CommonOps_DDRM.identity(3, 3), 0, 0, 3, 3, -1.0 / MathTools.square(omega));

      CommonOps_DDRM.transpose(B, BTranspose);
   }

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);

   public void computeEquivalentCostValues(double momentumRateWeight, double vrpTrackingWeight)
   {
      /*
        A' S1 + S1 A - Nb' R1inv Nb + Q1 = S1dot = 0
        or
        A1 S1 + S1 A - S1' B R1inv B' S1 - N R1inv N' + Q1
        If the standard CARE is formed as
        A' P + P A - P B R^-1 B' P + Q = 0
        then we can rewrite this as
                     S1 = P
         A - B R1inv N' = A
                      B = B
        Q1 - N R1inv N' = Q
      */

      MatrixTools.setDiagonal(Q, vrpTrackingWeight);
      MatrixTools.setDiagonal(R, momentumRateWeight);

      NativeCommonOps.multQuad(C, Q, Q1);
      NativeCommonOps.multQuad(D, Q, R1);
      CommonOps_DDRM.addEquals(R1, R);

      NativeCommonOps.invert(R1, R1Inverse);

      CommonOps_DDRM.mult(D, Q, DQ);

      tempMatrix.reshape(3, 3);
      CommonOps_DDRM.mult(Q, D, tempMatrix);
      CommonOps_DDRM.multTransA(C, tempMatrix, N);
      CommonOps_DDRM.transpose(N, NTranspose);
   }


   public void computeS2ConstantStateMatrices(DMatrixRMaj S1)
   {
      // Nb = N' + B' S1
      CommonOps_DDRM.transpose(N, Nb);
      CommonOps_DDRM.multAddTransA(B, S1, Nb);

      // A2 = Nb' R1inv B' - A'
      tempMatrix.reshape(3, 6);
      MatrixTools.scaleTranspose(-1.0, A, A2);
      CommonOps_DDRM.multTransB(R1Inverse, B, tempMatrix);
      CommonOps_DDRM.multAddTransA(Nb, tempMatrix, A2);

      // B2 = 2 (C' - Nb' R1inv D) Q
      CommonOps_DDRM.mult(R1Inverse, DQ, R1InverseDQ);
      CommonOps_DDRM.multTransA(-2.0, Nb, R1InverseDQ, B2);
      CommonOps_DDRM.multAddTransA(2.0, C, Q, B2);

      NativeCommonOps.invert(A2, A2Inverse);
      CommonOps_DDRM.mult(A2Inverse, B2, A2InverseB2);

      CommonOps_DDRM.multTransB(-0.5, R1Inverse, B, R1InverseBTranspose);
   }


   public DMatrixRMaj getA()
   {
      return A;
   }

   public DMatrixRMaj getB()
   {
      return B;
   }

   public DMatrixRMaj getC()
   {
      return C;
   }

   public DMatrixRMaj getD()
   {
      return D;
   }

   public DMatrixRMaj getQ()
   {
      return Q;
   }

   public DMatrixRMaj getR()
   {
      return R;
   }

   public DMatrixRMaj getDQ()
   {
      return DQ;
   }

   public DMatrixRMaj getNTranspose()
   {
      return NTranspose;
   }

   public DMatrixRMaj getQ1()
   {
      return Q1;
   }

   public DMatrixRMaj getR1()
   {
      return R1;
   }

   public DMatrixRMaj getR1Inverse()
   {
      return R1Inverse;
   }

   public DMatrixRMaj getA2()
   {
      return A2;
   }

   public DMatrixRMaj getA2Inverse()
   {
      return A2Inverse;
   }

   public DMatrixRMaj getA2InverseB2()
   {
      return A2InverseB2;
   }

   public DMatrixRMaj getNb()
   {
      return Nb;
   }
}
