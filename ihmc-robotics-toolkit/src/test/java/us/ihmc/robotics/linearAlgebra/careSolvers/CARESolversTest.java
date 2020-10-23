package us.ihmc.robotics.linearAlgebra.careSolvers;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.matrixlib.NativeCommonOps;

public class CARESolversTest
{
   private static final double epsilon = 1e-4;

   private List<CARESolver> getSolvers()
   {
      List<CARESolver> solvers = new ArrayList<>();
      solvers.add(new EigenvectorCARESolver());
      solvers.add(new NewtonCARESolver(new EigenvectorCARESolver()));
      solvers.add(new Newton2CARESolver(new EigenvectorCARESolver()));
      solvers.add(new DefectCorrectionCARESolver(new EigenvectorCARESolver()));

      solvers.add(new SignFunctionCARESolver());
      solvers.add(new NewtonCARESolver(new SignFunctionCARESolver()));
      solvers.add(new Newton2CARESolver(new SignFunctionCARESolver()));
      solvers.add(new DefectCorrectionCARESolver(new SignFunctionCARESolver()));

      return solvers;
   }

   @Test
   public void testSimple()
   {
      for (CARESolver solver : getSolvers())
      {
         DMatrixRMaj A = CommonOps_DDRM.identity(2);
         DMatrixRMaj B = CommonOps_DDRM.identity(2);
         DMatrixRMaj Q = CommonOps_DDRM.identity(2);
         DMatrixRMaj R = CommonOps_DDRM.identity(2);

         DMatrixRMaj AInput = new DMatrixRMaj(A);
         DMatrixRMaj BInput = new DMatrixRMaj(B);
         DMatrixRMaj QInput = new DMatrixRMaj(Q);
         DMatrixRMaj RInput = new DMatrixRMaj(R);

         solver.setMatrices(A, B, CommonOps_DDRM.identity(2), CommonOps_DDRM.identity(2), Q, R, null);
         solver.computeP();

         DMatrixRMaj assembledQ = new DMatrixRMaj(2, 2);
         CommonOps_DDRM.multTransA(A, solver.getP(), assembledQ);
         CommonOps_DDRM.multAdd(solver.getP(), A, assembledQ);

         DMatrixRMaj RInv = new DMatrixRMaj(2, 2);
         NativeCommonOps.invert(R, RInv);
         DMatrixRMaj BTransposeP = new DMatrixRMaj(2, 2);
         CommonOps_DDRM.multTransA(B, solver.getP(), BTransposeP);
         DMatrixRMaj BRInv = new DMatrixRMaj(2, 2);
         CommonOps_DDRM.mult(B, RInv, BRInv);
         DMatrixRMaj PBRInv = new DMatrixRMaj(2, 2);
         CommonOps_DDRM.mult(solver.getP(), BRInv, PBRInv);

         DMatrixRMaj PBRInvBTransposeP = new DMatrixRMaj(2, 2);
         CommonOps_DDRM.mult(PBRInv, BTransposeP, PBRInvBTransposeP);

         CommonOps_DDRM.addEquals(assembledQ, -1.0, PBRInvBTransposeP);

         CommonOps_DDRM.scale(-1.0, assembledQ);

         EjmlUnitTests.assertEquals(AInput, A, epsilon);
         EjmlUnitTests.assertEquals(BInput, B, epsilon);
         EjmlUnitTests.assertEquals(QInput, Q, epsilon);
         EjmlUnitTests.assertEquals(RInput, R, epsilon);

         assertIsSymmetric(solver.getP(), epsilon);
         assertSolutionIsValid(AInput, BInput, QInput, RInput, solver.getP(), epsilon);
      }
   }

   @Test
   public void testMatlabCare()
   {
      for (CARESolver solver : getSolvers())
      {
         int n = 2;
         int m = 1;
         DMatrixRMaj A = new DMatrixRMaj(n, n);
         DMatrixRMaj B = new DMatrixRMaj(n, m);
         DMatrixRMaj C = new DMatrixRMaj(1, 2);
         DMatrixRMaj Q = new DMatrixRMaj(2, 2);
         DMatrixRMaj R = new DMatrixRMaj(1, 1);
         A.set(0, 0, -3);
         A.set(0, 1, 2);
         A.set(1, 0, 1);
         A.set(1, 1, 1);
         B.set(1, 0, 1);
         C.set(0, 0, 1);
         C.set(0, 1, -1);
         R.set(0, 0, 3);

         CommonOps_DDRM.multInner(C, Q);

         solver.setMatrices(A, B, CommonOps_DDRM.identity(n), CommonOps_DDRM.identity(n), Q, R, null);
         solver.computeP();

         DMatrixRMaj PExpected = new DMatrixRMaj(2, 2);
         PExpected.set(0, 0, 0.5895);
         PExpected.set(0, 1, 1.8216);
         PExpected.set(1, 0, 1.8216);
         PExpected.set(1, 1, 8.8188);

         DMatrixRMaj P = solver.getP();

         //      assertIsSymmetric(P);
         assertSolutionIsValid(A, B, Q, R, P, epsilon);
         EjmlUnitTests.assertEquals(PExpected, P, 1e-4);
      }
   }

   @Test
   public void testMatlabCare2()
   {
      for (CARESolver solver : getSolvers())
      {
         int n = 3;
         int m = 1;
         DMatrixRMaj A = new DMatrixRMaj(n, n);
         DMatrixRMaj B = new DMatrixRMaj(n, m);
         DMatrixRMaj C = new DMatrixRMaj(1, n);
         DMatrixRMaj E = CommonOps_DDRM.identity(n);
         DMatrixRMaj Q = new DMatrixRMaj(n, n);
         DMatrixRMaj R = new DMatrixRMaj(1, 1);
         A.set(0, 0, 1);
         A.set(0, 1, -2);
         A.set(0, 2, 3);
         A.set(1, 0, -4);
         A.set(1, 1, 5);
         A.set(1, 2, 6);
         A.set(2, 0, 7);
         A.set(2, 1, 8);
         A.set(2, 2, 9);

         B.set(0, 0, 5);
         B.set(1, 0, 6);
         B.set(2, 0, -7);
         C.set(0, 0, 7);
         C.set(0, 1, -8);
         C.set(0, 2, 9);
         R.set(0, 0, 1);

         CommonOps_DDRM.multInner(C, Q);

         solver.setMatrices(A, B, CommonOps_DDRM.identity(n), E, Q, R, null);
         solver.computeP();

         DMatrixRMaj RInverse = new DMatrixRMaj(m, m);
         DMatrixRMaj BTranspose = new DMatrixRMaj(m, n);
         DMatrixRMaj M = new DMatrixRMaj(n, n);

         NativeCommonOps.invert(R, RInverse);
         CommonOps_DDRM.transpose(B, BTranspose);
         NativeCommonOps.multQuad(BTranspose, RInverse, M);

         //      assertIsSymmetric(solver.getP());
         assertSolutionIsValid(A, B, Q, R, solver.getP(), epsilon);
      }
   }

   private static void assertIsSymmetric(DMatrixRMaj A, double epsilon)
   {
      for (int row = 0; row < A.getNumRows(); row++)
      {
         for (int col = 0; col < A.getNumCols(); col++)
         {
            assertEquals("Not symmetric!", A.get(row, col), A.get(col, row), epsilon);
         }
      }
   }

   static void assertSolutionIsValid(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj Q, DMatrixRMaj R, DMatrixRMaj P, double epsilon)
   {
      int n = A.getNumRows();
      int m = B.getNumCols();
      DMatrixRMaj PDot = new DMatrixRMaj(n, n);
      DMatrixRMaj M = new DMatrixRMaj(m, m);
      DMatrixRMaj BTranspose = new DMatrixRMaj(m, n);
      CommonOps_DDRM.transpose(B, BTranspose);

      CARETools.computeM(BTranspose, R, null, M);
      CARETools.computeRiccatiRate(P, A, Q, M, PDot);

      EjmlUnitTests.assertEquals(new DMatrixRMaj(n, n), PDot, epsilon);
   }
}
