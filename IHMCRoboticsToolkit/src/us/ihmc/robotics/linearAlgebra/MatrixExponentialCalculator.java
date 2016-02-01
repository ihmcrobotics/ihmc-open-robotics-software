package us.ihmc.robotics.linearAlgebra;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.NormOps;
import org.ejml.ops.SpecializedOps;

public class MatrixExponentialCalculator
{
   private final int size;
   private final DenseMatrix64F As, As_2, As_4, As_6;
   private final DenseMatrix64F U;
   private final DenseMatrix64F V;
   private final DenseMatrix64F AV;
   private final DenseMatrix64F N, D;
   private final DenseMatrix64F temp;
   private final LinearSolver<DenseMatrix64F> solver;

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
      this.As = new DenseMatrix64F(size, size);
      this.As_2 = new DenseMatrix64F(size, size);
      this.As_4 = new DenseMatrix64F(size, size);
      this.As_6 = new DenseMatrix64F(size, size);

      this.U = new DenseMatrix64F(size, size);
      this.V = new DenseMatrix64F(size, size);

      this.AV = new DenseMatrix64F(size, size);
      this.N = new DenseMatrix64F(size, size);
      this.D = new DenseMatrix64F(size, size);

      this.temp = new DenseMatrix64F(size, size);

      solver = LinearSolverFactory.linear(size);
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
   public void compute(DenseMatrix64F result, DenseMatrix64F A)
   {
      MatrixTools.checkMatrixDimensions(A, size, size);

      int j = Math.max(0, 1 + (int) Math.floor(Math.log(NormOps.normPInf(A)) / Math.log(2)));

      As.set(A);
      CommonOps.scale(1.0 / Math.pow(2, j), As);    // scaled version of A

      // calculate D and N using special Horner techniques
      CommonOps.mult(As, As, As_2);
      CommonOps.mult(As_2, As_2, As_4);
      CommonOps.mult(As_4, As_2, As_6);

      // U = c0*I + c2*A^2 + c4*A^4 + (c6*I + c8*A^2 + c10*A^4 + c12*A^6)*A^6
      CommonOps.fill(U, 0.0);
      SpecializedOps.addIdentity(U, U, c0);
      CommonOps.addEquals(U, c2, As_2);
      CommonOps.addEquals(U, c4, As_4);

      CommonOps.fill(temp, 0.0);
      SpecializedOps.addIdentity(temp, temp, c6);
      CommonOps.addEquals(temp, c8, As_2);
      CommonOps.addEquals(temp, c10, As_4);
      CommonOps.addEquals(temp, c12, As_6);

      CommonOps.multAdd(temp, As_6, U);

      // V = c1*I + c3*A^2 + c5*A^4 + (c7*I + c9*A^2 + c11*A^4 + c13*A^6)*A^6
      CommonOps.fill(V, 0.0);
      SpecializedOps.addIdentity(V, V, c1);
      CommonOps.addEquals(V, c3, As_2);
      CommonOps.addEquals(V, c5, As_4);

      CommonOps.fill(temp, 0.0);
      SpecializedOps.addIdentity(temp, temp, c7);
      CommonOps.addEquals(temp, c9, As_2);
      CommonOps.addEquals(temp, c11, As_4);
      CommonOps.addEquals(temp, c13, As_6);

      CommonOps.multAdd(temp, As_6, V);

      CommonOps.mult(As, V, AV);
      CommonOps.add(U, AV, N);
      CommonOps.subtract(U, AV, D);

      // solve DF = N for F
      solver.setA(D);
      solver.solve(N, result);

      // now square j times
      for (int k = 0; k < j; k++)
      {
         temp.set(result);
         CommonOps.mult(temp, temp, result);
      }
   }
}
