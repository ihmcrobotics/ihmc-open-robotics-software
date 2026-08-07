package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.lieGroup.SE3LieGroupTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Tests for {@link SEK3Utils}, the SE_k(3) exp/log utilities.
 *
 * <p>The two core properties checked here:
 * <ul>
 *   <li><b>Round-trip:</b> for random ξ and several k, log(exp(ξ)) == ξ.</li>
 *   <li><b>SE(3) agreement:</b> for k = 1, exp(ξ) reproduces the trusted
 *       {@link SE3LieGroupTools#exp} result block-for-block.</li>
 * </ul>
 * </p>
 */
public class SEK3UtilsTest
{
   private static final double EPSILON = 1.0e-10;
   private static final int ITERATIONS = 1000;

   /**
    * Round-trip test: for k = 1, 2, 3 and random ξ = [φ; v₁; …; v_k],
    * applying exp then log must recover the original ξ to within EPSILON.
    */

   @Test
   public void testExpLogRoundTrip()
   {
      Random random = new Random(1234L);

      for (int k = 1; k <= 3; k++)
      {
         double[] xi = new double[3 + 3 * k];
         double[] recovered = new double[3 + 3 * k];
         DMatrixRMaj X = new DMatrixRMaj(3 + k, 3+ k);

         for (int iter = 0; iter < ITERATIONS; iter++)
         {
            randomAlgebraVector(random, k, xi);

            SEK3Utils.exp(xi, X);
            SEK3Utils.log(X, recovered);

            for (int j = 0; j < xi.length; j++)
               assertEquals(xi[j], recovered[j], EPSILON);
         }
      }
   }

   /**
    * SE(3) agreement test: for k = 1, the 4×4 matrix from {@code SEK3_Utils.exp}
    * must equal the {@link RigidBodyTransform} from {@code SE3LieGroupTools.exp}.
    */
   @Test
   public void testExpMatchesSE3ForK1()
   {
      Random random = new Random(5768L);
      double[] xi = new double[6];
      DMatrixRMaj X = new DMatrixRMaj(4,4);
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int iter = 0; iter < ITERATIONS; iter++)
      {
         randomAlgebraVector(random, 1, xi);

         SEK3Utils.exp(xi, X);
         SE3LieGroupTools.exp(xi, transform);

         // Rotation block: compare the top-left 3x3.
         for (int r = 0; r < 3; r++)
            for (int c = 0; c <3; c++)
               assertEquals(transform.getRotation().getElement(r,c), X.get(r,c), EPSILON);

         // Translation column: SE(3) t == column 3 of X
         assertEquals(transform.getTranslation().getX(), X.get(0,3), EPSILON);
         assertEquals(transform.getTranslation().getY(), X.get(1,3), EPSILON);
         assertEquals(transform.getTranslation().getZ(), X.get(2,3), EPSILON);
      }
   }

   /**
    * Validation test: {@code log} must reject an output array whose length does not
    * match the matrix size. A 5×5 X is k = 2 and expects a length-9 ξ; passing a
    * length-6 array must throw {@link IllegalArgumentException}.
    */
   @Test
   public void testLogRejectsWrongSizedOutput()
   {
      DMatrixRMaj X = new DMatrixRMaj(5,5);
      double[] wrongLength = new double[6];

      assertThrows(IllegalArgumentException.class, () -> SEK3Utils.log(X, wrongLength));
   }

   /**
    * Adjoint k = 1 agreement: for a random SE(3) element, {@code SEK3_Utils.adjoint} must
    * equal the 6×6 {@link SE3LieGroupTools#adjoint} block-for-block (same [φ; ρ] ordering).
    */
   @Test
   public void testAdjointMatchesSE3ForK1()
   {
      Random random = new Random(7777L);
      double[] xi = new double[6];
      DMatrixRMaj X = new DMatrixRMaj(4,4);
      DMatrixRMaj adj = new DMatrixRMaj(6,6);
      DMatrixRMaj adjSE3 = new DMatrixRMaj(6,6);
      RigidBodyTransform transform = new RigidBodyTransform();

      for (int iter = 0; iter < ITERATIONS; iter++)
      {
         randomAlgebraVector(random, 1, xi);

         SEK3Utils.exp(xi, X);
         SEK3Utils.adjoint(X, adj);

         SE3LieGroupTools.exp(xi, transform);
         SE3LieGroupTools.adjoint(transform, adjSE3);

         for (int r = 0; r < 6; r++)
            for (int c = 0; c < 6; c++)
               assertEquals(adjSE3.get(r,c), adj.get(r,c), EPSILON);
      }
   }

   /**
    * Adjoint homomorphism: Ad_{X_A·X_B} = Ad_{X_A}·Ad_{X_B}, for k = 1, 2, 3.
    * Build X_A, X_B via exp, form X_AB by matrix multiplication, and compare adjoint(X_AB)
    * to adjoint(X_A)·adjoint(X_B).
    */
   @Test
   public void testAdjointHomomorphism()
   {
      Random random = new Random(8888L);

      for (int k = 1; k <= 3; k++)
      {
         int n = 3 + k;
         int m = 3 + 3 * k;
         double[] xiA = new double[m];
         double[] xiB = new double[m];
         DMatrixRMaj XA = new DMatrixRMaj(n,n);
         DMatrixRMaj XB = new DMatrixRMaj(n,n);
         DMatrixRMaj XAB = new DMatrixRMaj(n,n);
         DMatrixRMaj adjA = new DMatrixRMaj(m,m);
         DMatrixRMaj adjB = new DMatrixRMaj(m,m);
         DMatrixRMaj adjAB = new DMatrixRMaj(m,m);
         DMatrixRMaj adjProduct = new DMatrixRMaj(m,m);

         for (int iter = 0; iter < ITERATIONS; iter++)
         {
            randomAlgebraVector(random, k, xiA);
            randomAlgebraVector(random, k, xiB);
            SEK3Utils.exp(xiA, XA);
            SEK3Utils.exp(xiB, XB);

            CommonOps_DDRM.mult(XA, XB, XAB);

            SEK3Utils.adjoint(XA, adjA);
            SEK3Utils.adjoint(XB, adjB);
            SEK3Utils.adjoint(XAB, adjAB);
            CommonOps_DDRM.mult(adjA,adjB, adjProduct);

            for (int r = 0; r < m; r++)
               for (int c = 0; c < m; c++)
                  assertEquals(adjProduct.get(r,c), adjAB.get(r,c), 1.0e-9);
         }
      }
   }

   /**
    * Adjoint defining identity: log(X·exp(ξ)·X⁻¹) == Ad_X·ξ, for k = 1, 2, 3.
    * This ties {@code adjoint} back to {@code exp}/{@code log} through conjugation.
    */
   @Test
   public void testAdjointConjugationIdentity()
   {
      Random random = new Random(9999L);

      for (int k = 1; k <= 3; k++)
      {
         int n = 3 + k;
         int m = 3 + 3 * k;
         double[] eta = new double[m];
         double[] xi = new double[m];
         double[] adXi = new double[m];
         double[] lhsXi = new double[m];

         DMatrixRMaj X = new DMatrixRMaj(n,n);
         DMatrixRMaj Xinv = new DMatrixRMaj(n,n);
         DMatrixRMaj expXi = new DMatrixRMaj(n,n);
         DMatrixRMaj temp = new DMatrixRMaj(n,n);
         DMatrixRMaj conj = new DMatrixRMaj(n,n);
         DMatrixRMaj adj = new DMatrixRMaj(m,m);

         for (int iter = 0; iter < ITERATIONS; iter++)
         {
            randomAlgebraVector(random, k, eta);
            randomAlgebraVector(random, k, xi);

            SEK3Utils.exp(eta, X);
            CommonOps_DDRM.invert(X, Xinv);

            SEK3Utils.exp(xi, expXi);
            CommonOps_DDRM.mult(X, expXi, temp);
            CommonOps_DDRM.mult(temp, Xinv, conj);
            SEK3Utils.log(conj, lhsXi);

            SEK3Utils.adjoint(X, adj);
            multiply(adj, xi, adXi);

            for (int j = 0; j < m; j++)
               assertEquals(adXi[j], lhsXi[j], 1.0e-8);
         }
      }
   }

   /**
    * Matrix-times-vector: resultToPack = A · x. (A is rows×cols, x and result length cols/rows.)
    *
    * @param A            the matrix. Not modified.
    * @param x            the input vector, length A.getNumCols(). Not modified.
    * @param resultToPack the output vector, length A.getNumRows(). Modified.
    */
   private static void multiply(DMatrixRMaj A, double[] x, double[] resultToPack)
   {
      int rows = A.getNumRows();
      int cols = A.getNumCols();
      for (int r = 0; r < rows; r++)
      {
         double sum = 0.0;
         for (int c = 0; c < cols; c++)
            sum += A.get(r, c) * x[c];
         resultToPack[r] = sum;
      }
   }

   /**
    * Fills {@code xiToPack} (length 3+3k) with a random algebra element:
    * a bounded rotation vector φ (‖φ‖ ≤ π) in slots 0..2, then k random ℝ³ vectors.
    *
    * @param random   the source of randomness.
    * @param k        the number of translation columns.
    * @param xiToPack the array to fill; must have length 3 + 3k. Modified.
    */
   private static void randomAlgebraVector(Random random, int k, double[] xiToPack)
   {
      Vector3D phi = EuclidCoreRandomTools.nextRotationVector(random);
      xiToPack[0] = phi.getX();
      xiToPack[1] = phi.getY();
      xiToPack[2] = phi.getZ();

      for (int i = 0; i < k; i++)
      {
         Vector3D v = EuclidCoreRandomTools.nextVector3D(random);
         xiToPack[3 + 3 * i] = v.getX();
         xiToPack[3 + 3 * i + 1] = v.getY();
         xiToPack[3 + 3 * i + 2] = v.getZ();
      }
   }
}
