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
 * Property tests for {@link InvariantEKF#marginalizeContact} (the liftoff half of the contact
 * add/remove lifecycle, paper eq 30), the complement of {@link InvariantEKF#reseedContact} (augment,
 * eq 31/32; tested in {@link InvariantEKFReseedTest}):
 * <ol>
 *   <li><b>PSD:</b> zeroing a contact's covariance rows/columns is a direct sum of the retained
 *       (unchanged, PSD) block with a zero block, so P⁺ stays symmetric PSD for any PSD P.</li>
 *   <li><b>Retained-block invariance:</b> marginalizing a contact leaves every covariance entry
 *       outside that contact's rows/columns identical, and zeros the contact's own rows/columns —
 *       exactly the Gaussian marginal (delete the variable, the rest is unchanged).</li>
 *   <li><b>Swing contributes nothing:</b> a stance-foot update after marginalizing the other foot
 *       cannot move the marginalized contact (zero gain, decoupled).</li>
 *   <li><b>Round-trip:</b> augment (reseed) after marginalize reconstructs the eq-32 block exactly,
 *       regardless of the zeroed intermediate.</li>
 *   <li><b>Base-Z drift (the motivating property):</b> keeping a swing foot in the state and
 *       updating it with a biased FK measurement drifts base Z off the stance-consistent baseline;
 *       marginalizing it instead reproduces the no-swing-foot baseline to machine precision.</li>
 * </ol>
 */
public class InvariantEKFMarginalizeTest
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

   /** Body-frame FK measurement consistent with a chosen world foothold: y = Rᵀ(foothold − p). */
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
   public void testMarginalizePreservesSymmetryAndPSD()
   {
      Random random = new Random(20260811);
      for (int trial = 0; trial < 50; trial++)
      {
         InvariantEKF ekf = randomlyInitializedEKF(random, randomPsd(random, 0.5));
         ekf.marginalizeContact(random.nextBoolean() ? 0 : 1);

         DMatrixRMaj covariance = ekf.getState().getCovariance();
         for (int r = 0; r < M; r++)
            for (int c = r + 1; c < M; c++)
               assertEquals(covariance.get(r, c), covariance.get(c, r), EPS, "P not symmetric at (" + r + "," + c + ")");
         assertTrue(minEigenvalue(covariance) > -1.0e-8, "P lost PSD, trial " + trial);
      }
   }

   @Test
   public void testMarginalizeLeavesRetainedBlockInvariantAndZerosContactCross()
   {
      Random random = new Random(13);
      InvariantEKF ekf = randomlyInitializedEKF(random, randomPsd(random, 0.7));
      DMatrixRMaj before = ekf.getState().getCovariance().copy();
      int dIdx = ekf.getState().contactTangentIndex(0);

      ekf.marginalizeContact(0);
      DMatrixRMaj after = ekf.getState().getCovariance();

      for (int r = 0; r < M; r++)
      {
         boolean rInContact = r >= dIdx && r < dIdx + 3;
         for (int c = 0; c < M; c++)
         {
            boolean cInContact = c >= dIdx && c < dIdx + 3;
            if (rInContact || cInContact)
               assertEquals(0.0, after.get(r, c), 0.0, "contact cross-entry not zeroed at (" + r + "," + c + ")");
            else
               assertEquals(before.get(r, c), after.get(r, c), 0.0, "retained entry changed at (" + r + "," + c + ")");
         }
      }
   }

   /**
    * A marginalized (swing) foot has zero gain in any subsequent update: it stays fully decoupled and gains no
    * information, so it can inject nothing into the base. (Note: in a right-invariant filter the correction acts
    * on the left, X⁺ = exp(−K·r)·X, so a nonzero rotation gain co-rotates the WHOLE frame — base and both contact
    * columns. The swing foot's world position therefore rides that shared rotation; what marks it "contributes
    * nothing" is its zero own-gain, i.e. its covariance rows/columns staying zero, not a frozen world position.)
    */
   @Test
   public void testMarginalizedContactStaysDecoupledThroughStanceUpdate()
   {
      Random random = new Random(555);
      InvariantEKF ekf = randomlyInitializedEKF(random, randomPsd(random, 0.5));
      ekf.marginalizeContact(0); // lift the left foot

      // A nonzero-residual update on the STANCE foot (contact 1): pulls base + contact 1.
      Matrix3D fkCovariance = new Matrix3D();
      fkCovariance.setIdentity();
      fkCovariance.scale(1.0e-4);
      Vector3D stanceMeasurement = consistentMeasurement(ekf, new Point3D(0.12, -0.14, 0.03));
      ekf.update(1, stanceMeasurement, fkCovariance);
      assertTrue(ekf.wasLastUpdateApplied(), "stance update should pass the conditioning gate");

      // The marginalized contact's covariance rows/columns are still zero: it gained no information, no coupling.
      DMatrixRMaj covariance = ekf.getState().getCovariance();
      int dIdx = ekf.getState().contactTangentIndex(0);
      for (int a = 0; a < 3; a++)
         for (int c = 0; c < M; c++)
         {
            assertEquals(0.0, covariance.get(dIdx + a, c), EPS, "marginalized contact regained a covariance row");
            assertEquals(0.0, covariance.get(c, dIdx + a), EPS, "marginalized contact regained a covariance column");
         }
   }

   @Test
   public void testAugmentAfterMarginalizeRecoversEq32Block()
   {
      Random random = new Random(808);
      InvariantEKF ekf = randomlyInitializedEKF(random, randomPsd(random, 0.5));
      DMatrixRMaj beforeMarginalize = ekf.getState().getCovariance().copy();
      int pIdx = ekf.getState().basePositionTangentIndex();
      int dIdx = ekf.getState().contactTangentIndex(0);

      RotationMatrix rotation = new RotationMatrix();
      ekf.getState().getRotation(rotation);
      Matrix3D fkCovariance = new Matrix3D();
      fkCovariance.setIdentity();
      fkCovariance.scale(3.0e-6);

      ekf.marginalizeContact(0);
      ekf.reseedContact(0, consistentMeasurement(ekf, new Point3D(0.1, 0.05, 0.0)), fkCovariance);

      Matrix3D rotatedN = new Matrix3D(fkCovariance);
      rotation.transform(rotatedN); // R·N·Rᵀ
      DMatrixRMaj after = ekf.getState().getCovariance();
      for (int a = 0; a < 3; a++)
         for (int b = 0; b < 3; b++)
         {
            // eq 32: P_dd = P_pp + R·N·Rᵀ. P_pp is untouched by marginalizing contact 0, so use the pre-marginalize value.
            assertEquals(beforeMarginalize.get(pIdx + a, pIdx + b) + rotatedN.getElement(a, b), after.get(dIdx + a, dIdx + b), EPS, "P_dd != P_pp + RNR^T");
            // P_theta,d = P_theta,p (theta block starts at 0)
            assertEquals(after.get(a, pIdx + b), after.get(a, dIdx + b), EPS, "P_theta,d != P_theta,p after round-trip");
         }
   }

   /**
    * The motivating property. Single support on the right foot (contact 1). A: keep the left (swing) foot in the
    * state and update it every tick with a biased FK measurement (world Z off by −ε), the leak the add/remove
    * mode removes. B: marginalize the swing foot each tick and skip its update. C: never touch the swing foot.
    * The base + stance evolution is independent of the swing contact, so B must match C to machine precision,
    * while A drifts its base Z below the stance-consistent baseline.
    */
   @Test
   public void testSingleSupportBaseZDriftAlwaysOnVersusAddRemove()
   {
      int ticks = 500;
      double dt = 1.0e-3;
      double epsilon = 5.0e-3;               // 5 mm downward bias in the swing FK measurement
      double baseContactVariance = 1.0e-6;

      InvariantEKF ekfA = staticStandingEKF(); // always-on: biased swing update
      InvariantEKF ekfB = staticStandingEKF(); // add-remove: marginalize swing
      InvariantEKF ekfC = staticStandingEKF(); // baseline: ignore swing

      Vector3D gravityCompensatingAccel = new Vector3D(0.0, 0.0, -GRAVITY); // R=I, static: a = −g so v,p stay put
      Vector3D zeroOmega = new Vector3D();

      Matrix3D stanceR = new Matrix3D();
      stanceR.setIdentity();
      stanceR.scale(1.0e-4);
      Matrix3D swingR = new Matrix3D();
      swingR.setIdentity();
      swingR.scale(1.0e-4 * 90.0); // swing-foot measurement inflation, as in the driver

      for (int t = 0; t < ticks; t++)
      {
         ekfA.predict(zeroOmega, gravityCompensatingAccel, dt);
         ekfB.predict(zeroOmega, gravityCompensatingAccel, dt);
         ekfC.predict(zeroOmega, gravityCompensatingAccel, dt);

         // Stance foot (contact 1): consistent (zero-residual) update on all three — the anchor.
         ekfA.update(1, consistentMeasurement(ekfA, stanceFoothold(ekfA)), stanceR);
         ekfB.update(1, consistentMeasurement(ekfB, stanceFoothold(ekfB)), stanceR);
         ekfC.update(1, consistentMeasurement(ekfC, stanceFoothold(ekfC)), stanceR);

         // Swing foot (contact 0):
         // A keeps it in the state, forgetful (inflated slip) and updated with a Z-biased measurement.
         ekfA.setContactSlipVariance(0, baseContactVariance * 90.0);
         Vector3D biasedSwing = consistentMeasurement(ekfA, swingFoothold(ekfA));
         biasedSwing.addZ(epsilon); // body-frame Z ≈ world Z (R≈I): a biased swing FK that pulls the base down
         ekfA.update(0, biasedSwing, swingR);
         // B marginalizes it out every tick; C leaves it untouched.
         ekfB.marginalizeContact(0);
      }

      Vector3D pA = new Vector3D();
      Vector3D pB = new Vector3D();
      Vector3D pC = new Vector3D();
      ekfA.getBasePosition(pA);
      ekfB.getBasePosition(pB);
      ekfC.getBasePosition(pC);

      // Add-remove == baseline (swing foot contributes nothing to base Z).
      assertEquals(pC.getZ(), pB.getZ(), 1.0e-9, "add-remove base Z must match the no-swing-foot baseline");
      // Baseline is the static standing height (no drift).
      assertEquals(1.0, pC.getZ(), 1.0e-6, "baseline base Z should hold the standing height");
      // Always-on drifts base Z downward off the baseline by a clearly measurable amount.
      assertTrue(pA.getZ() < pC.getZ() - 1.0e-4,
                 "always-on should drift base Z downward (Z_A=" + pA.getZ() + ", Z_C=" + pC.getZ() + ")");
   }

   private static InvariantEKF staticStandingEKF()
   {
      InvariantEKF ekf = new InvariantEKF(N_CONTACTS, 1.0e-6, 1.0e-5, 1.0e-6, GRAVITY);
      RotationMatrix rotation = new RotationMatrix(); // identity
      Point3D basePosition = new Point3D(0.0, 0.0, 1.0);
      Point3D[] contacts = new Point3D[] {new Point3D(0.0, 0.1, 0.0),   // left / swing
                                          new Point3D(0.0, -0.1, 0.0)}; // right / stance
      DMatrixRMaj p0 = new DMatrixRMaj(M, M);
      for (int i = 0; i < M; i++)
         p0.set(i, i, 1.0e-4);
      ekf.initialize(rotation, new Vector3D(), basePosition, contacts, p0);
      return ekf;
   }

   private static Point3D swingFoothold(InvariantEKF ekf)
   {
      Vector3D d = new Vector3D();
      ekf.getContactPosition(0, d);
      return new Point3D(d);
   }

   private static Point3D stanceFoothold(InvariantEKF ekf)
   {
      Vector3D d = new Vector3D();
      ekf.getContactPosition(1, d);
      return new Point3D(d);
   }
}
