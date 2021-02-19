 package us.ihmc.commonWalkingControlModules.modelPredictiveController.tools;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.omg.PortableInterceptor.ACTIVE;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;

 public class EfficientMatrixExponentialCalculator
 {
    private int rows;
    private int Bcols;
    private int Ccols;

    private final DMatrixRMaj temp = new DMatrixRMaj(0, 0);
    private final LinearSolverDense<DMatrixRMaj> solver;

    private final DMatrixRMaj As = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Bs = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Cs = new DMatrixRMaj(0, 0);

    private final DMatrixRMaj As_2 = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Bs_2 = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Cs_2 = new DMatrixRMaj(0, 0);

    private final DMatrixRMaj As_4 = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Bs_4 = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Cs_4 = new DMatrixRMaj(0, 0);

    private final DMatrixRMaj As_6 = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Bs_6 = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Cs_6 = new DMatrixRMaj(0, 0);

    private final DMatrixRMaj U_A = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj U_B = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj U_C = new DMatrixRMaj(0, 0);

    private final DMatrixRMaj V_A = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj V_B = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj V_C = new DMatrixRMaj(0, 0);

    private final DMatrixRMaj N_A = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj N_B = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj N_C = new DMatrixRMaj(0, 0);

    private final DMatrixRMaj N_Ainverse = new DMatrixRMaj(0, 0);

    private final DMatrixRMaj D_A = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj D_B = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj D_C = new DMatrixRMaj(0, 0);

    private final DMatrixRMaj tempBd = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj tempCd = new DMatrixRMaj(0, 0);

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

       solver = LinearSolverFactory_DDRM.linear(rows);
    }

    public void reshape(int rows, int Bcols, int Ccols)
    {
       this.rows = rows;
       this.Bcols = Bcols;
       this.Ccols = Ccols;

       As.reshape(rows, rows);
       Bs.reshape(rows, Bcols);
       Cs.reshape(rows, Ccols);

       As_2.reshape(rows, rows);
       Bs_2.reshape(rows, Bcols);
       Cs_2.reshape(rows, Ccols);

       As_4.reshape(rows, rows);
       Bs_4.reshape(rows, Bcols);
       Cs_4.reshape(rows, Ccols);

       As_6.reshape(rows, rows);
       Bs_6.reshape(rows, Bcols);
       Cs_6.reshape(rows, Ccols);

       U_A.reshape(rows, rows);
       U_B.reshape(rows, Bcols);
       U_C.reshape(rows, Ccols);

       V_A.reshape(rows, rows);
       V_B.reshape(rows, Bcols);
       V_C.reshape(rows, Ccols);

       N_A.reshape(rows, rows);
       N_B.reshape(rows, Bcols);
       N_C.reshape(rows, Ccols);

       N_Ainverse.reshape(rows, rows);

       Ainternal.reshape(rows, rows);
       Binternal.reshape(rows, Bcols);
       Cinternal.reshape(rows, Ccols);

       Adinternal.reshape(rows, rows);
       Bdinternal.reshape(rows, Bcols);
       Cdinternal.reshape(rows, Ccols);

       tempBd.reshape(rows, Bcols);
       tempCd.reshape(rows, Ccols);
    }

    private final DMatrixRMaj Ainternal = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Binternal = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Cinternal = new DMatrixRMaj(0, 0);

    private final DMatrixRMaj Adinternal = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Bdinternal = new DMatrixRMaj(0, 0);
    private final DMatrixRMaj Cdinternal = new DMatrixRMaj(0, 0);

    public void compute(DMatrixRMaj result, DMatrixRMaj A)
    {
       MatrixTools.setMatrixBlock(Ainternal, 0, 0, A, 0, 0, rows, rows, 1.0);
       MatrixTools.setMatrixBlock(Binternal, 0, 0, A, 0, rows, rows, Bcols, 1.0);
       MatrixTools.setMatrixBlock(Cinternal, 0, 0, A, 0, rows + Bcols, rows, Ccols, 1.0);

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

       As.set(A);
       Bs.set(B);
       Cs.set(C);
       CommonOps_DDRM.scale(1.0 / Math.pow(2, j), As);    // scaled version of A
       CommonOps_DDRM.scale(1.0 / Math.pow(2, j), Bs);    // scaled version of B
       CommonOps_DDRM.scale(1.0 / Math.pow(2, j), Cs);    // scaled version of C

       // calculate D and N using special Horner techniques
       CommonOps_DDRM.mult(As, As, As_2);
       CommonOps_DDRM.mult(As, Bs, Bs_2);
       CommonOps_DDRM.mult(As, Cs, Cs_2);

       CommonOps_DDRM.mult(As_2, As_2, As_4);
       CommonOps_DDRM.mult(As_2, Bs_2, Bs_4);
       CommonOps_DDRM.mult(As_2, Cs_2, Cs_4);

       CommonOps_DDRM.mult(As_4, As_2, As_6);
       CommonOps_DDRM.mult(As_4, Bs_2, Bs_6);
       CommonOps_DDRM.mult(As_4, Cs_2, Cs_6);

       // U = c0*I + c2*A^2 + c4*A^4 + (c6*I + c8*A^2 + c10*A^4 + c12*A^6)*A^6
       MatrixTools.setDiagonal(U_A, c0);
       CommonOps_DDRM.addEquals(U_A, c2, As_2);
       CommonOps_DDRM.addEquals(U_A, c4, As_4);

       U_B.zero();
       CommonOps_DDRM.addEquals(U_B, c2, Bs_2);
       CommonOps_DDRM.addEquals(U_B, c4, Bs_4);

       U_C.zero();
       CommonOps_DDRM.addEquals(U_C, c2, Cs_2);
       CommonOps_DDRM.addEquals(U_C, c4, Cs_4);

       temp.reshape(rows, rows);
       MatrixTools.setDiagonal(temp, c6);
       CommonOps_DDRM.addEquals(temp, c8, As_2);
       CommonOps_DDRM.addEquals(temp, c10, As_4);
       CommonOps_DDRM.addEquals(temp, c12, As_6);

       CommonOps_DDRM.multAdd(temp, As_6, U_A);
       CommonOps_DDRM.multAdd(temp, Bs_6, U_B);
       CommonOps_DDRM.multAdd(temp, Cs_6, U_C);


       // V = c1*I + c3*A^2 + c5*A^4 + (c7*I + c9*A^2 + c11*A^4 + c13*A^6)*A^6
       MatrixTools.setDiagonal(V_A, c1);
       CommonOps_DDRM.addEquals(V_A, c3, As_2);
       CommonOps_DDRM.addEquals(V_A, c5, As_4);

       V_B.zero();
       CommonOps_DDRM.addEquals(V_B, c3, Bs_2);
       CommonOps_DDRM.addEquals(V_B, c5, Bs_4);

       V_C.zero();
       CommonOps_DDRM.addEquals(V_C, c3, Cs_2);
       CommonOps_DDRM.addEquals(V_C, c5, Cs_4);

       temp.reshape(rows, rows);

       MatrixTools.setDiagonal(temp, c7);
       CommonOps_DDRM.addEquals(temp, c9, As_2);
       CommonOps_DDRM.addEquals(temp, c11, As_4);
       CommonOps_DDRM.addEquals(temp, c13, As_6);

       CommonOps_DDRM.multAdd(temp, As_6, V_A);
       CommonOps_DDRM.multAdd(temp, Bs_6, V_B);
       CommonOps_DDRM.multAdd(temp, Cs_6, V_C);

       N_A.set(U_A);
       CommonOps_DDRM.multAdd(As, V_A, N_A);

       CommonOps_DDRM.add(U_B, c1, Bs, N_B);
       CommonOps_DDRM.multAdd(As, V_B, N_B);

       CommonOps_DDRM.add(U_C, c1, Cs, N_C);
       CommonOps_DDRM.multAdd(As, V_C, N_C);

       D_A.set(U_A);
       CommonOps_DDRM.multAdd(-1.0, As, V_A, D_A);

       CommonOps_DDRM.add(U_B, -c1, Bs, D_B);
       CommonOps_DDRM.multAdd(-1.0, As, V_B, D_B);

       CommonOps_DDRM.add(U_C, -c1, Cs, D_C);
       CommonOps_DDRM.multAdd(-1.0, As, V_C, D_C);

       // solve DF = N for F
       solver.setA(N_A);
       solver.invert(N_Ainverse);

       CommonOps_DDRM.mult(N_Ainverse, D_A, Ad);

       temp.reshape(rows, Bcols);
       CommonOps_DDRM.subtract(D_B, N_B, temp);
       CommonOps_DDRM.mult(N_Ainverse, temp, Bd);

       temp.reshape(rows, Ccols);
       CommonOps_DDRM.subtract(D_C, N_C, temp);
       CommonOps_DDRM.mult(N_Ainverse, temp, Cd);

       // now square j times
       for (int k = 0; k < j; k++)
       {
          temp.set(Ad);
          tempBd.set(Bd);
          tempCd.set(Cd);
          MatrixTools.addDiagonal(temp, 1.0);

          CommonOps_DDRM.mult(temp, tempBd, Bd);
          CommonOps_DDRM.mult(temp, tempCd, Cd);

          MatrixTools.addDiagonal(temp, -1.0);
          CommonOps_DDRM.mult(temp, temp, Ad);
       }
    }

    public static double inducedPInf(DMatrixRMaj A , DMatrixRMaj B, DMatrixRMaj C)
    {
       double max = 0;

       int m = A.numRows;
       int An = A.numCols;
       int Bn = B.numCols;
       int Cn = C.numCols;

       for( int i = 0; i < m; i++ ) {
          double total = 0;
          for( int j = 0; j < An; j++ ) {
             total += Math.abs(A.get(i,j));
          }
          for( int j = 0; j < Bn; j++ ) {
             total += Math.abs(B.get(i,j));
          }
          for( int j = 0; j < Cn; j++ ) {
             total += Math.abs(C.get(i,j));
          }
          if( total > max ) {
             max = total;
          }
       }

       return max;
    }
 }
