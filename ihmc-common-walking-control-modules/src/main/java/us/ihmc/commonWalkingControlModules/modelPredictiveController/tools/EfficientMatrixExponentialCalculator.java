package us.ihmc.commonWalkingControlModules.modelPredictiveController.tools;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;

/**
 * This class is the same as the matrix exponential. However, it assumes that the state matrix can be segmented into blocks,
 * where the bottom rows are all zero. That makes it much more efficient to compute.
 */
public class EfficientMatrixExponentialCalculator
{
   private int rows;
   private int Bcols;
   private int Ccols;

   private final NativeMatrix temp = new NativeMatrix(0, 0);

   private final NativeMatrix As = new NativeMatrix(0, 0);
   private final NativeMatrix Bs = new NativeMatrix(0, 0);
   private final NativeMatrix Cs = new NativeMatrix(0, 0);

   private final NativeMatrix As_2 = new NativeMatrix(0, 0);
   private final NativeMatrix Bs_2 = new NativeMatrix(0, 0);
   private final NativeMatrix Cs_2 = new NativeMatrix(0, 0);

   private final NativeMatrix As_4 = new NativeMatrix(0, 0);
   private final NativeMatrix Bs_4 = new NativeMatrix(0, 0);
   private final NativeMatrix Cs_4 = new NativeMatrix(0, 0);

   private final NativeMatrix As_6 = new NativeMatrix(0, 0);
   private final NativeMatrix Bs_6 = new NativeMatrix(0, 0);
   private final NativeMatrix Cs_6 = new NativeMatrix(0, 0);

   private final NativeMatrix U_A = new NativeMatrix(0, 0);
   private final NativeMatrix U_B = new NativeMatrix(0, 0);
   private final NativeMatrix U_C = new NativeMatrix(0, 0);

   private final NativeMatrix V_A = new NativeMatrix(0, 0);
   private final NativeMatrix V_B = new NativeMatrix(0, 0);
   private final NativeMatrix V_C = new NativeMatrix(0, 0);

   private final NativeMatrix N_A = new NativeMatrix(0, 0);
   private final NativeMatrix N_B = new NativeMatrix(0, 0);
   private final NativeMatrix N_C = new NativeMatrix(0, 0);

   private final NativeMatrix D_Ainverse = new NativeMatrix(0, 0);

   private final NativeMatrix D_A = new NativeMatrix(0, 0);
   private final NativeMatrix D_B = new NativeMatrix(0, 0);
   private final NativeMatrix D_C = new NativeMatrix(0, 0);

   private final NativeMatrix nativeAd = new NativeMatrix(0, 0);
   private final NativeMatrix nativeBd = new NativeMatrix(0, 0);
   private final NativeMatrix nativeCd = new NativeMatrix(0, 0);

   // constants for pade approximation
   private static final double c0 = 1.0;
   private static final double c1 = 0.5;
   private static final double c2 = 0.12;
   private static final double c3 = 0.01833333333333333;
   private static final double c4 = 0.0019927536231884053;
   private static final double c5 = 1.630434782608695E-4;
   private static final double c6 = 1.0351966873706E-5;
   private static final double c7 = 5.175983436853E-7;
   private static final double c8 = 2.0431513566525E-8;
   private static final double c9 = 6.306022705717593E-10;
   private static final double c10 = 1.4837700484041396E-11;
   private static final double c11 = 2.5291534915979653E-13;
   private static final double c12 = 2.8101705462199615E-15;
   private static final double c13 = 1.5440497506703084E-17;

   public EfficientMatrixExponentialCalculator(int rows, int Bcols, int Ccols)
   {
      reshape(rows, Bcols, Ccols);
   }

   public void reshape(int rows, int Bcols, int Ccols)
   {
      this.rows = rows;
      this.Bcols = Bcols;
      this.Ccols = Ccols;

      Ainternal.reshape(rows, rows);
      Binternal.reshape(rows, Bcols);
      Cinternal.reshape(rows, Ccols);

      Adinternal.reshape(rows, rows);
      Bdinternal.reshape(rows, Bcols);
      Cdinternal.reshape(rows, Ccols);
   }

   private final DMatrixRMaj Ainternal = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Binternal = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Cinternal = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj Adinternal = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Bdinternal = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Cdinternal = new DMatrixRMaj(0, 0);

   public void compute(DMatrixRMaj result, DMatrixRMaj A)
   {
      CommonOps_DDRM.extract(A, 0, rows, 0, rows, Ainternal);
      CommonOps_DDRM.extract(A, 0, rows, rows, rows + Bcols, Binternal);
      CommonOps_DDRM.extract(A, 0, rows, rows + Bcols, rows + Bcols + Ccols, Cinternal);

      compute(Ainternal, Binternal, Cinternal, Adinternal, Bdinternal, Cdinternal);

      result.zero();
      MatrixTools.setMatrixBlock(result, 0, 0, Adinternal, 0, 0, rows, rows, 1.0);
      MatrixTools.setMatrixBlock(result, 0, rows, Bdinternal, 0, 0, rows, Bcols, 1.0);
      MatrixTools.setMatrixBlock(result, 0, rows + Bcols, Cdinternal, 0, 0, rows, Ccols, 1.0);

      int size = rows + Bcols + Ccols;
      for (int row = rows; row < size; row++)
         result.set(row, row, 1.0);
   }

   public void compute(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C, DMatrixRMaj Ad, DMatrixRMaj Bd, DMatrixRMaj Cd)
   {
      MatrixTools.checkMatrixDimensions(A, rows, rows);
      MatrixTools.checkMatrixDimensions(B, rows, Bcols);
      MatrixTools.checkMatrixDimensions(C, rows, Ccols);
      MatrixTools.checkMatrixDimensions(Ad, rows, rows);
      MatrixTools.checkMatrixDimensions(Bd, rows, Bcols);
      MatrixTools.checkMatrixDimensions(Cd, rows, Ccols);

      int j = Math.max(0, 1 + (int) Math.floor(Math.log(inducedPInf(A, B, C)) / Math.log(2)));
      double inverseJ = 1.0 / Math.pow(2, j);

      As.scale(inverseJ, A); // scaled version of A
      Bs.scale(inverseJ, B); // scaled version of B
      Cs.scale(inverseJ, C); // scaled version of C

      // calculate D and N using special Horner techniques
      As_2.mult(As, As);
      Bs_2.mult(As, Bs);
      Cs_2.mult(As, Cs);

      As_4.mult(As_2, As_2);
      Bs_4.mult(As_2, Bs_2);
      Cs_4.mult(As_2, Cs_2);

      As_6.mult(As_4, As_2);
      Bs_6.mult(As_4, Bs_2);
      Cs_6.mult(As_4, Cs_2);

      // U = c0*I + c2*A^2 + c4*A^4 + (c6*I + c8*A^2 + c10*A^4 + c12*A^6)*A^6
      U_A.add(c2, As_2, c4, As_4);
      U_A.addDiagonal(c0);

      U_B.add(c2, Bs_2, c4, Bs_4);

      U_C.add(c2, Cs_2, c4, Cs_4);

      temp.add(c8, As_2, c10, As_4);
      temp.addEquals(c12, As_6);
      temp.addDiagonal(c6);

      U_A.multAdd(temp, As_6);
      U_B.multAdd(temp, Bs_6);
      U_C.multAdd(temp, Cs_6);

      // V = c1*I + c3*A^2 + c5*A^4 + (c7*I + c9*A^2 + c11*A^4 + c13*A^6)*A^6
      V_A.add(c3, As_2, c5, As_4);
      V_A.addDiagonal(c1);

      V_B.add(c3, Bs_2, c5, Bs_4);

      V_C.add(c3, Cs_2, c5, Cs_4);

      temp.add(c9, As_2, c11, As_4);
      temp.addEquals(c13, As_6);
      temp.addDiagonal(c7);

      V_A.multAdd(temp, As_6);
      V_B.multAdd(temp, Bs_6);
      V_C.multAdd(temp, Cs_6);

      N_A.set(U_A);
      N_A.multAdd(As, V_A);

      N_B.add(U_B, c1, Bs);
      N_B.multAdd(As, V_B);

      N_C.add(U_C, c1, Cs);
      N_C.multAdd(As, V_C);

      D_A.set(U_A);
      D_A.multAdd(-1.0, As, V_A);

      D_B.add(U_B, -c1, Bs);
      D_B.multAdd(-1.0, As, V_B);

      D_C.add(U_C, -c1, Cs);
      D_C.multAdd(-1.0, As, V_C);

      // solve DF = N for F
      D_Ainverse.invert(D_A);

      nativeAd.mult(D_Ainverse, N_A);

      temp.subtract(N_B, D_B);
      nativeBd.mult(D_Ainverse, temp);

      temp.subtract(N_C, D_C);
      nativeCd.mult(D_Ainverse, temp);

      // now square j times
      for (int k = 0; k < j; k++)
      {
         nativeBd.multAdd(nativeAd, nativeBd);
         nativeCd.multAdd(nativeAd, nativeCd);
         nativeAd.mult(nativeAd, nativeAd);
      }

      nativeAd.get(Ad);
      nativeBd.get(Bd);
      nativeCd.get(Cd);
   }

   public static double inducedPInf(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C)
   {
      double max = 0;

      int m = A.numRows;
      int An = A.numCols;
      int Bn = B.numCols;
      int Cn = C.numCols;

      for (int i = 0; i < m; i++)
      {
         double total = 0;
         for (int j = 0; j < An; j++)
         {
            total += Math.abs(A.get(i, j));
         }
         for (int j = 0; j < Bn; j++)
         {
            total += Math.abs(B.get(i, j));
         }
         for (int j = 0; j < Cn; j++)
         {
            total += Math.abs(C.get(i, j));
         }
         if (total > max)
         {
            max = total;
         }
      }

      return max;
   }
}
