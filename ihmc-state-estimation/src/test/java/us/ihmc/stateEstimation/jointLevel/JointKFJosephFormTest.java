package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.jupiter.api.Test;

/**
 * {@link JointKFUpdate#josephCovarianceUpdate} must equal the textbook Joseph form (I-KH)P(I-KH)^T + K R K^T for
 * any gain, not only the optimal one: that is what makes it a Joseph form rather than the P - KHP shortcut.
 */
public class JointKFJosephFormTest
{
   private static final int DIM = 42; // Alex: 9 joints (q, qd) + 8 IMU biases

   @Test
   public void testMatchesTheTextbookJosephFormForOptimalAndPerturbedGains()
   {
      Random random = new Random(20261005L);
      for (int k : new int[] {9, 27})
      {
         for (int trial = 0; trial < 50; trial++)
         {
            DMatrixRMaj P = randomSpd(random, DIM, 1.0e-3);
            DMatrixRMaj H = RandomMatrices_DDRM.rectangle(k, DIM, -1.0, 1.0, random);
            if (k == 27)
               for (int row = 0; row < k; row++)
                  for (int column = 0; column < 9; column++)
                     H.set(row, column, 0.0); // the stacked gyro rows do not observe q
            DMatrixRMaj R = randomSpd(random, k, 1.0e-4);

            DMatrixRMaj PHt = new DMatrixRMaj(DIM, k);
            CommonOps_DDRM.multTransB(P, H, PHt);
            DMatrixRMaj S = new DMatrixRMaj(k, k);
            CommonOps_DDRM.mult(H, PHt, S);
            CommonOps_DDRM.addEquals(S, R);
            DMatrixRMaj Sinv = S.copy();
            assertTrue(CommonOps_DDRM.invert(Sinv));
            DMatrixRMaj K = new DMatrixRMaj(DIM, k);
            CommonOps_DDRM.mult(PHt, Sinv, K);
            if (trial % 2 == 1) // a gain off the optimum by a few percent
               for (int i = 0; i < K.getNumElements(); i++)
                  K.data[i] *= 1.0 + 0.05 * (random.nextDouble() - 0.5);

            DMatrixRMaj expected = textbookJoseph(P, H, R, K);
            DMatrixRMaj actual = P.copy();
            JointKFUpdate.josephCovarianceUpdate(actual, PHt, K, S, new DMatrixRMaj(DIM, k));

            double scale = CommonOps_DDRM.elementMaxAbs(expected);
            DMatrixRMaj difference = actual.copy();
            CommonOps_DDRM.subtractEquals(difference, expected);
            double error = CommonOps_DDRM.elementMaxAbs(difference) / scale;
            assertTrue(error < 1.0e-11, "k=" + k + " trial " + trial + ": relative difference " + error);
         }
      }
   }

   @Test
   public void testSparseProductsMatchDense()
   {
      Random random = new Random(7L);
      for (int trial = 0; trial < 50; trial++)
      {
         int k = trial % 2 == 0 ? 9 : 27;
         DMatrixRMaj P = randomSpd(random, DIM, 1.0e-3);
         DMatrixRMaj H = new DMatrixRMaj(k, DIM);
         for (int r = 0; r < k; r++) // a few nonzeros per row, like the encoder and gyro-pair rows
            for (int t = 0; t < 1 + random.nextInt(8); t++)
               H.set(r, random.nextInt(DIM), random.nextGaussian());
         int[][] nonzeros = new int[k][DIM];
         int[] counts = new int[k];
         for (int r = 0; r < k; r++)
            for (int c = 0; c < DIM; c++)
               if (H.get(r, c) != 0.0)
                  nonzeros[r][counts[r]++] = c;

         DMatrixRMaj expected = new DMatrixRMaj(DIM, k), actual = new DMatrixRMaj(DIM, k);
         CommonOps_DDRM.multTransB(P, H, expected);
         JointKFUpdate.sparseMultTransB(P, H, nonzeros, counts, actual);
         assertClose(expected, actual, 1.0e-13, "P H^T");

         DMatrixRMaj expectedS = new DMatrixRMaj(k, k), actualS = new DMatrixRMaj(k, k);
         CommonOps_DDRM.mult(H, expected, expectedS);
         JointKFUpdate.sparseMult(H, expected, nonzeros, counts, actualS);
         assertClose(expectedS, actualS, 1.0e-13, "H P H^T");
      }
   }

   @Test
   public void testStructuredPropagationMatchesDenseTransition()
   {
      Random random = new Random(11L);
      int n = 9;
      double dt = 0.001;
      DMatrixRMaj F = CommonOps_DDRM.identity(DIM);
      for (int i = 0; i < n; i++)
         F.set(i, n + i, dt);
      for (int trial = 0; trial < 20; trial++)
      {
         DMatrixRMaj P = randomSpd(random, DIM, 1.0e-3);
         DMatrixRMaj x = RandomMatrices_DDRM.rectangle(DIM, 1, -1.0, 1.0, random);
         DMatrixRMaj expectedP = new DMatrixRMaj(DIM, DIM), tmp = new DMatrixRMaj(DIM, DIM), expectedX = new DMatrixRMaj(DIM, 1);
         CommonOps_DDRM.mult(F, P, tmp);
         CommonOps_DDRM.multTransB(tmp, F, expectedP);
         CommonOps_DDRM.mult(F, x, expectedX);
         JointKFPrediction.propagate(x, P, n, dt, new DMatrixRMaj(DIM, DIM));
         assertClose(expectedP, P, 1.0e-13, "F P F^T");
         assertClose(expectedX, x, 1.0e-15, "F x");
      }
   }

   private static void assertClose(DMatrixRMaj expected, DMatrixRMaj actual, double relativeTolerance, String what)
   {
      DMatrixRMaj difference = actual.copy();
      CommonOps_DDRM.subtractEquals(difference, expected);
      double error = CommonOps_DDRM.elementMaxAbs(difference) / Math.max(CommonOps_DDRM.elementMaxAbs(expected), 1.0e-300);
      assertTrue(error < relativeTolerance, what + ": relative difference " + error);
   }

   private static DMatrixRMaj textbookJoseph(DMatrixRMaj P, DMatrixRMaj H, DMatrixRMaj R, DMatrixRMaj K)
   {
      int dim = P.getNumRows();
      DMatrixRMaj IKH = CommonOps_DDRM.identity(dim);
      CommonOps_DDRM.multAdd(-1.0, K, H, IKH);
      DMatrixRMaj tmp = new DMatrixRMaj(dim, dim);
      CommonOps_DDRM.mult(IKH, P, tmp);
      DMatrixRMaj result = new DMatrixRMaj(dim, dim);
      CommonOps_DDRM.multTransB(tmp, IKH, result);
      DMatrixRMaj KR = new DMatrixRMaj(dim, R.getNumCols());
      CommonOps_DDRM.mult(K, R, KR);
      CommonOps_DDRM.multAddTransB(KR, K, result);
      return result;
   }

   private static DMatrixRMaj randomSpd(Random random, int n, double floor)
   {
      DMatrixRMaj A = RandomMatrices_DDRM.rectangle(n, n, -1.0, 1.0, random);
      DMatrixRMaj spd = new DMatrixRMaj(n, n);
      CommonOps_DDRM.multTransB(A, A, spd);
      CommonOps_DDRM.scale(1.0 / n, spd);
      for (int i = 0; i < n; i++)
         spd.add(i, i, floor);
      return spd;
   }
}
