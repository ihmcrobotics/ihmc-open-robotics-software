package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.EigenDecomposition;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;

import java.util.Random;

public abstract class CARESolverTest
{
   private static final double epsilon = 1e-7;

   protected abstract CARESolver getSolver();

   @Test
   public void testSimple()
   {
      DenseMatrix64F A = CommonOps.identity(2);
      DenseMatrix64F B = CommonOps.identity(2);
      DenseMatrix64F Q = CommonOps.identity(2);
      DenseMatrix64F R = CommonOps.identity(2);

      DenseMatrix64F AInput = new DenseMatrix64F(A);
      DenseMatrix64F BInput = new DenseMatrix64F(B);
      DenseMatrix64F QInput = new DenseMatrix64F(Q);
      DenseMatrix64F RInput = new DenseMatrix64F(R);

      CARESolver solver = getSolver();
      solver.setMatrices(A, B, Q, R);
      solver.computeP();

      DenseMatrix64F assembledQ = new DenseMatrix64F(2, 2);
      CommonOps.multTransA(A, solver.getP(), assembledQ);
      CommonOps.multAdd(solver.getP(), A, assembledQ);

      DenseMatrix64F RInv = new DenseMatrix64F(2, 2);
      NativeCommonOps.invert(R, RInv);
      DenseMatrix64F BTransposeP = new DenseMatrix64F(2, 2);
      CommonOps.multTransA(B, solver.getP(), BTransposeP);
      DenseMatrix64F BRInv = new DenseMatrix64F(2, 2);
      CommonOps.mult(B, RInv, BRInv);
      DenseMatrix64F PBRInv = new DenseMatrix64F(2, 2);
      CommonOps.mult(solver.getP(), BRInv, PBRInv);

      DenseMatrix64F PBRInvBTransposeP = new DenseMatrix64F(2, 2);
      CommonOps.mult(PBRInv, BTransposeP, PBRInvBTransposeP);

      CommonOps.addEquals(assembledQ, -1.0, PBRInvBTransposeP);

      CommonOps.scale(-1.0, assembledQ);

      EjmlUnitTests.assertEquals(AInput, A, epsilon);
      EjmlUnitTests.assertEquals(BInput, B, epsilon);
      EjmlUnitTests.assertEquals(QInput, Q, epsilon);
      EjmlUnitTests.assertEquals(RInput, R, epsilon);

      EjmlUnitTests.assertEquals(Q, assembledQ, epsilon);
   }

   @Test
   public void testRandom()
   {
      CARESolver solver = getSolver();
      Random random = new Random(1738L);
      for (int iter = 0; iter < 1000; iter++)
      {
         int size = RandomNumbers.nextInt(random, 1, 10);

         // FIXME need to guarantee A and B are controllable

         DenseMatrix64F A = CARESolverTestTools.generateRandomSymmetricPDMatrix(random, size);
         DenseMatrix64F B = CARESolverTestTools.generateRandomSymmetricPDMatrix(random, size);
         DenseMatrix64F Q = CARESolverTestTools.generateRandomDiagonalMatrix(random, size, true);
         DenseMatrix64F R = CARESolverTestTools.generateRandomDiagonalMatrix(random, size, false);

         EigenDecomposition<DenseMatrix64F> eigenDecomposition = DecompositionFactory.eig(size, false);
         eigenDecomposition.decompose(A);
         for (int i = 0; i < size; i++)
         {
            if (eigenDecomposition.getEigenvalue(i).getReal() <= 0.0)
               throw new RuntimeException("BOOO");
         }

         assertIsControllable(A, B);

         DenseMatrix64F ATranspose = new DenseMatrix64F(A);
         DenseMatrix64F BTranspose = new DenseMatrix64F(B);
         CommonOps.transpose(ATranspose);
         CommonOps.transpose(BTranspose);
         DenseMatrix64F M = new DenseMatrix64F(size, size);
         DenseMatrix64F RinvBTranspose = new DenseMatrix64F(size, size);
         DenseMatrix64F Rinv = new DenseMatrix64F(size, size);
         NativeCommonOps.invert(R, Rinv);
         CommonOps.mult(Rinv, BTranspose, RinvBTranspose);
         CommonOps.mult(B, RinvBTranspose, M);

         DenseMatrix64F hamiltonianExpected = new DenseMatrix64F(2 * size, 2 * size);
         DenseMatrix64F hamiltonian = new DenseMatrix64F(2 * size, 2 * size);
         MatrixTools.setMatrixBlock(hamiltonianExpected, 0, 0, A, 0, 0, size, size, 1.0);
         MatrixTools.setMatrixBlock(hamiltonianExpected, 0, size, M, 0, 0, size, size, -1.0);
         MatrixTools.setMatrixBlock(hamiltonianExpected, size, 0, Q, 0, 0, size, size, -1.0);
         MatrixTools.setMatrixBlock(hamiltonianExpected, size, size, ATranspose, 0, 0, size, size, -1.0);

         solver.setMatrices(A, B, Q, R);

         if (solver instanceof HamiltonianCARESolver)
         {
            ((HamiltonianCARESolver) solver).assembleHamiltonian(hamiltonian);
            EjmlUnitTests.assertEquals(hamiltonianExpected, hamiltonian, epsilon);
         }

         solver.computeP();

         DenseMatrix64F assembledQ = new DenseMatrix64F(size, size);
         CommonOps.multTransA(A, solver.getP(), assembledQ);
         CommonOps.multAdd(solver.getP(), A, assembledQ);

         DenseMatrix64F RInv = new DenseMatrix64F(size, size);
         NativeCommonOps.invert(R, RInv);
         DenseMatrix64F BTransposeP = new DenseMatrix64F(size, size);
         CommonOps.multTransA(B, solver.getP(), BTransposeP);
         DenseMatrix64F BRInv = new DenseMatrix64F(size, size);
         CommonOps.mult(B, RInv, BRInv);
         DenseMatrix64F PBRInv = new DenseMatrix64F(size, size);
         CommonOps.mult(solver.getP(), BRInv, PBRInv);

         DenseMatrix64F PBRInvBTransposeP = new DenseMatrix64F(size, size);
         CommonOps.mult(PBRInv, BTransposeP, PBRInvBTransposeP);

         CommonOps.addEquals(assembledQ, -1.0, PBRInvBTransposeP);

         CommonOps.scale(-1.0, assembledQ);

         EjmlUnitTests.assertEquals(Q, assembledQ, epsilon);
      }
   }

   private static void assertIsControllable(DenseMatrix64F A, DenseMatrix64F B)
   {
      int n = A.getNumRows();
      int r = B.getNumCols();

      DenseMatrix64F tempMatrix = new DenseMatrix64F(B);
      DenseMatrix64F R = new DenseMatrix64F(n, n * r);
      MatrixTools.setMatrixBlock(R, 0, 0, tempMatrix, 0, 0, n, r, 1.0);
      for (int i = 1; i < n - 1; i++)
      {
         DenseMatrix64F tempMatrix2 = new DenseMatrix64F(n, r);
         CommonOps.mult(A, tempMatrix, tempMatrix2);

         MatrixTools.setMatrixBlock(R, 0, i * r, tempMatrix2, 0, 0, n, r, 1.0);

         tempMatrix.set(tempMatrix2);

      }

      SingularValueDecomposition<DenseMatrix64F> decomposer = DecompositionFactory.svd(n, n * r, false, false, false);
      decomposer.decompose(R);
      if (MathTools.min(decomposer.getSingularValues()) < 1e-12)
         throw new RuntimeException("System is not controllable.");
   }
}
