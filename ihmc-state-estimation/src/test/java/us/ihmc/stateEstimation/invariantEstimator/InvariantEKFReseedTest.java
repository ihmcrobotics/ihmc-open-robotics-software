package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Property tests for {@link InvariantEKF#reseedContact} (H4 Phase 2 touchdown re-seed, see the
 * 2026-07-16 derivation note):
 * <ol>
 *   <li>PSD preservation: the covariance clone P_{d·}←P_{p·}, P_{dd}←P_{pp}+R·N·Rᵀ is a congruence
 *       plus a PSD addition, so P⁺ must stay symmetric PSD for any PSD P.</li>
 *   <li>Consistency: after re-seed, P_{dd} = P_{pp} + R·N·Rᵀ and P_{θd} = P_{θp} exactly.</li>
 *   <li>Zero-release: a contact update with the SAME measurement immediately after re-seed has
 *       zero residual, hence zero NIS and zero applied correction (K_θ rows vanish).</li>
 * </ol>
 */
public class InvariantEKFReseedTest
{
   private static final int N_CONTACTS = 2;
   private static final int M = 9 + 3 * N_CONTACTS;
   private static final double EPS = 1.0e-10;
   private static final double GRAVITY = -9.81;

   private static InvariantEKF randomlyInitializedEKF(Random random, DMatrixRMaj covarianceToUse)
   {
      InvariantEKF ekf = new InvariantEKF(N_CONTACTS, 1.0e-4, 1.0e-3, 1.0e-6, GRAVITY);
      RotationMatrix rotation = new RotationMatrix();
      rotation.setYawPitchRoll(random.nextDouble() - 0.5, 0.4 * (random.nextDouble() - 0.5), 0.4 * (random.nextDouble() - 0.5));
      Point3D basePosition = new Point3D(random.nextDouble(), random.nextDouble(), 0.9 + 0.1 * random.nextDouble());
      Point3D[] contacts = new Point3D[N_CONTACTS];
      for (int i = 0; i < N_CONTACTS; i++)
         contacts[i] = new Point3D(basePosition.getX() + 0.3 * (random.nextDouble() - 0.5), basePosition.getY() + (i == 0 ? 0.1 : -0.1), 0.0);
      ekf.initialize(rotation, new Vector3D(), basePosition, contacts, covarianceToUse);
      return ekf;
   }

   private static DMatrixRMaj randomPsd(Random random, double scale)
   {
      DMatrixRMaj a = new DMatrixRMaj(M, M);
      for (int i = 0; i < M * M; i++)
         a.set(i, scale * (random.nextDouble() - 0.5));
      DMatrixRMaj psd = new DMatrixRMaj(M, M);
      CommonOps_DDRM.multTransB(a, a, psd); // A·Aᵀ ⪰ 0
      for (int i = 0; i < M; i++)
         psd.add(i, i, 1.0e-9);
      return psd;
   }

   private static double minEigenvalue(DMatrixRMaj symmetric)
   {
      EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(M, false, true);
      assertTrue(eig.decompose(symmetric.copy()));
      double min = Double.POSITIVE_INFINITY;
      for (int i = 0; i < eig.getNumberOfEigenvalues(); i++)
         min = Math.min(min, eig.getEigenvalue(i).getReal());
      return min;
   }

   /** Body-frame FK measurement consistent with a chosen world foothold: y = Rᵀ(d − p). */
   private static Vector3D consistentMeasurement(InvariantEKF ekf, Point3D worldFoothold)
   {
      RotationMatrix rotation = new RotationMatrix();
      Vector3D basePosition = new Vector3D();
      ekf.getState().getRotation(rotation);
      ekf.getState().getBasePosition(basePosition);
      Vector3D y = new Vector3D(worldFoothold);
      y.sub(basePosition);
      rotation.inverseTransform(y);
      return y;
   }

   @Test
   public void testReseedPreservesPositiveSemiDefiniteness()
   {
      Random random = new Random(4242);
      for (int trial = 0; trial < 50; trial++)
      {
         InvariantEKF ekf = randomlyInitializedEKF(random, randomPsd(random, 0.5));
         Matrix3D fkCovariance = new Matrix3D();
         fkCovariance.setIdentity();
         fkCovariance.scale(1.0e-6 * (1.0 + random.nextDouble()));

         // Re-seed onto a foothold displaced by a swing-scale discrepancy (cm).
         Point3D newFoothold = new Point3D(0.05 * random.nextDouble(), 0.05 * random.nextDouble(), 0.01 * random.nextDouble());
         ekf.reseedContact(0, consistentMeasurement(ekf, newFoothold), fkCovariance);

         DMatrixRMaj covariance = ekf.getState().getCovariance();
         // symmetric
         for (int r = 0; r < M; r++)
            for (int c = r + 1; c < M; c++)
               assertEquals(covariance.get(r, c), covariance.get(c, r), EPS, "P not symmetric at (" + r + "," + c + ")");
         assertTrue(minEigenvalue(covariance) > -1.0e-8, "P lost PSD, trial " + trial);
      }
   }

   @Test
   public void testReseedCovarianceConsistency()
   {
      Random random = new Random(99);
      InvariantEKF ekf = randomlyInitializedEKF(random, randomPsd(random, 0.5));
      DMatrixRMaj before = ekf.getState().getCovariance().copy();
      int pIdx = ekf.getState().basePositionTangentIndex();
      int dIdx = ekf.getState().contactTangentIndex(0);

      RotationMatrix rotation = new RotationMatrix();
      ekf.getState().getRotation(rotation);
      Matrix3D fkCovariance = new Matrix3D();
      fkCovariance.setIdentity();
      fkCovariance.scale(3.0e-6);
      ekf.reseedContact(0, consistentMeasurement(ekf, new Point3D(0.1, 0.05, 0.0)), fkCovariance);

      Matrix3D rotatedN = new Matrix3D(fkCovariance);
      rotation.transform(rotatedN); // R·N·Rᵀ
      DMatrixRMaj after = ekf.getState().getCovariance();
      for (int a = 0; a < 3; a++)
         for (int b = 0; b < 3; b++)
         {
            assertEquals(before.get(pIdx + a, pIdx + b) + rotatedN.getElement(a, b), after.get(dIdx + a, dIdx + b), EPS, "P_dd != P_pp + RNR^T");
            // rotation-to-contact cross equals rotation-to-base cross (K_theta = 0 condition), theta block starts at 0
            assertEquals(after.get(a, pIdx + b), after.get(a, dIdx + b), EPS, "P_theta,d != P_theta,p");
         }
   }

   @Test
   public void testZeroReleaseAfterReseed()
   {
      Random random = new Random(7);
      InvariantEKF ekf = randomlyInitializedEKF(random, randomPsd(random, 0.5));
      Matrix3D fkCovariance = new Matrix3D();
      fkCovariance.setIdentity();
      fkCovariance.scale(1.0e-4);

      Vector3D measurement = consistentMeasurement(ekf, new Point3D(0.08, -0.03, 0.005));
      double preResidual = ekf.reseedContact(0, measurement, fkCovariance);
      assertTrue(preResidual > 1.0e-3, "test should exercise a nonzero geometric discrepancy");

      // Same measurement immediately after: residual must be ~0, update must not move the state.
      RotationMatrix rotationBefore = new RotationMatrix();
      Vector3D positionBefore = new Vector3D();
      ekf.getState().getRotation(rotationBefore);
      ekf.getState().getBasePosition(positionBefore);

      ekf.update(0, measurement, fkCovariance);
      assertTrue(ekf.wasLastUpdateApplied(), "update should pass the conditioning gate");
      assertEquals(0.0, ekf.getLastNormalizedInnovationSquared(), 1.0e-9, "NIS after re-seed should be ~0");
      assertEquals(0.0, ekf.getLastCorrectionRotationNorm(), 1.0e-9, "rotation release after re-seed should be ~0");

      RotationMatrix rotationAfter = new RotationMatrix();
      Vector3D positionAfter = new Vector3D();
      ekf.getState().getRotation(rotationAfter);
      ekf.getState().getBasePosition(positionAfter);
      assertTrue(rotationAfter.epsilonEquals(rotationBefore, 1.0e-9), "base rotation moved on a zero-residual update");
      assertTrue(positionAfter.epsilonEquals(positionBefore, 1.0e-9), "base position moved on a zero-residual update");
   }
}
