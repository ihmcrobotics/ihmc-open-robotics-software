package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.SpecializedOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.matrixlib.MatrixTools;

public class MatrixExponentialCalculator
{
   private final int size;
   private final DMatrixRMaj As, As_2, As_4, As_6;
   private final DMatrixRMaj U;
   private final DMatrixRMaj V;
   private final DMatrixRMaj AV;
   private final DMatrixRMaj N, D;
   private final DMatrixRMaj temp;
   private final LinearSolverDense<DMatrixRMaj> solver;

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

   public MatrixExponentialCalculator(int size)
   {
      this.size = size;
      this.As = new DMatrixRMaj(size, size);
      this.As_2 = new DMatrixRMaj(size, size);
      this.As_4 = new DMatrixRMaj(size, size);
      this.As_6 = new DMatrixRMaj(size, size);

      this.U = new DMatrixRMaj(size, size);
      this.V = new DMatrixRMaj(size, size);

      this.AV = new DMatrixRMaj(size, size);
      this.N = new DMatrixRMaj(size, size);
      this.D = new DMatrixRMaj(size, size);

      this.temp = new DMatrixRMaj(size, size);

      solver = LinearSolverFactory_DDRM.linear(size);
   }

   /**
    * Adapted from jblas
    * Original documentation:
    *
    * Calculate matrix exponential of a square matrix.
    *
    * A scaled Pade approximation algorithm is used.
    * The algorithm has been directly translated from Golub & Van Loan "Matrix Computations",
    * algorithm 11.3.1. Special Horner techniques from 11.2 are also used to minimize the number
    * of matrix multiplications.
    *
    * @param A square matrix
    * @return matrix exponential of A
    */
   public void compute(DMatrixRMaj result, DMatrixRMaj A)
   {
      MatrixTools.checkMatrixDimensions(A, size, size);

      int j = Math.max(0, 1 + (int) Math.floor(Math.log(NormOps_DDRM.normPInf(A)) / Math.log(2)));

      As.set(A);
      CommonOps_DDRM.scale(1.0 / Math.pow(2, j), As);    // scaled version of A

      // calculate D and N using special Horner techniques
      CommonOps_DDRM.mult(As, As, As_2);
      CommonOps_DDRM.mult(As_2, As_2, As_4);
      CommonOps_DDRM.mult(As_4, As_2, As_6);

      // U = c0*I + c2*A^2 + c4*A^4 + (c6*I + c8*A^2 + c10*A^4 + c12*A^6)*A^6
      CommonOps_DDRM.fill(U, 0.0);
      SpecializedOps_DDRM.addIdentity(U, U, c0);
      CommonOps_DDRM.addEquals(U, c2, As_2);
      CommonOps_DDRM.addEquals(U, c4, As_4);

      CommonOps_DDRM.fill(temp, 0.0);
      SpecializedOps_DDRM.addIdentity(temp, temp, c6);
      CommonOps_DDRM.addEquals(temp, c8, As_2);
      CommonOps_DDRM.addEquals(temp, c10, As_4);
      CommonOps_DDRM.addEquals(temp, c12, As_6);

      CommonOps_DDRM.multAdd(temp, As_6, U);

      // V = c1*I + c3*A^2 + c5*A^4 + (c7*I + c9*A^2 + c11*A^4 + c13*A^6)*A^6
      CommonOps_DDRM.fill(V, 0.0);
      SpecializedOps_DDRM.addIdentity(V, V, c1);
      CommonOps_DDRM.addEquals(V, c3, As_2);
      CommonOps_DDRM.addEquals(V, c5, As_4);

      CommonOps_DDRM.fill(temp, 0.0);
      SpecializedOps_DDRM.addIdentity(temp, temp, c7);
      CommonOps_DDRM.addEquals(temp, c9, As_2);
      CommonOps_DDRM.addEquals(temp, c11, As_4);
      CommonOps_DDRM.addEquals(temp, c13, As_6);

      CommonOps_DDRM.multAdd(temp, As_6, V);

      CommonOps_DDRM.mult(As, V, AV);
      CommonOps_DDRM.add(U, AV, N);
      CommonOps_DDRM.subtract(U, AV, D);

      // solve DF = N for F
      solver.setA(D);
      solver.solve(N, result);

      // now square j times
      for (int k = 0; k < j; k++)
      {
         temp.set(result);
         CommonOps_DDRM.mult(temp, temp, result);
      }
   }
}
