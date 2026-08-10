package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertPositiveSemiDefinite;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.assertSymmetric;
import static us.ihmc.stateEstimation.jointLevel.JointLevelKFTestFixture.block;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Property tests for the LIVE tier of {@link JointKFParameters}: the tuning that is re-read on the hot path so
 * the calibration procedures documented on those constants can be iterated in SCS instead of through a
 * recompile-reflash-rerun cycle.
 *
 * <p>A YoVariable that is published but silently ignored is worse than a compile-time constant — it invites a
 * retune that does nothing. These tests pin the three properties that make "LIVE" a real claim:</p>
 * <ol>
 *   <li><b>Liveness</b> — writing the parameter changes the filter's behavior on the very next {@code predict()},
 *   with no re-construction;</li>
 *   <li><b>Direction</b> — it moves the right way, so the knob is physically meaningful and not just wired;</li>
 *   <li><b>Exact reversibility</b> — restoring the value restores the previous numbers BIT-FOR-BIT (tolerance
 *   0.0), which is what rules out hysteresis, a latched first read, or a stale cached copy;</li>
 * </ol>
 * plus the invariant that must survive any retune: Qa stays symmetric and PSD, because it is a covariance and
 * the Joseph update's conditioning gates depend on it.
 */
public class JointKFLiveParameterTest
{
   private static final double DT = JointLevelKFTestFixture.DT;

   /**
    * The per-joint reflected rotor inertia (jointKFParam_rotorInertia_&lt;joint&gt;) is added to the Schur
    * complement's diagonal BEFORE inversion: Lambda_eff = Lambda + diag(rotor). Raising it therefore stiffens
    * the torque -&gt; acceleration map Lambda_eff^-1 and must LOWER every joint's process noise.
    */
   @Test
   public void testPerJointRotorInertiaIsLiveDirectionalAndExactlyReversible()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairMassMatrix(9100L, 8, 1, 7);
      assertTrue(f.filter.isUsingMassMatrixProcessNoise(), "fixture must be on the Schur path for this test to mean anything");
      int n = f.n;
      f.filter.setStateForTest(new DMatrixRMaj(f.dim, 1), JointLevelKFTestFixture.spd(f.dim, 91L));

      double[] q = new double[n];
      for (int i = 0; i < n; i++)
         q[i] = -0.6 + 0.15 * i;
      f.applyConsistentMotion(q, new double[n]);

      f.filter.predict();
      DMatrixRMaj qddBaseline = block(f.filter.getProcessNoise(), n, n, n, n); // qd-qd block = dt * Qa
      assertSymmetric(qddBaseline, 0.0, "Qa is exactly symmetric by construction (Y Yᵀ)");
      assertPositiveSemiDefinite(qddBaseline, "Qa PSD at the default rotor inertia");

      // Record the defaults, then double every joint's rotor inertia.
      YoDouble[] rotor = new YoDouble[n];
      double[] defaults = new double[n];
      for (int i = 0; i < n; i++)
      {
         String jointName = f.filteredJoints.get(i).getName();
         rotor[i] = (YoDouble) f.registry.findVariable("jointKFParam_rotorInertia_" + jointName);
         assertNotNull(rotor[i], "jointKFParam_rotorInertia_" + jointName + " must be published");
         defaults[i] = rotor[i].getValue();
         assertTrue(defaults[i] > 0.0, "rotor inertia default must be positive");
         rotor[i].set(2.0 * defaults[i]);
      }

      f.filter.predict();
      DMatrixRMaj qddStiffer = block(f.filter.getProcessNoise(), n, n, n, n);
      assertSymmetric(qddStiffer, 0.0, "Qa still exactly symmetric after the live retune");
      assertPositiveSemiDefinite(qddStiffer, "Qa still PSD after the live retune");

      // Liveness + direction: more reflected rotor inertia => stiffer Lambda_eff => strictly less process noise
      // on EVERY joint. A merely-published-but-ignored parameter fails this on the first joint.
      for (int i = 0; i < n; i++)
      {
         assertTrue(qddStiffer.get(i, i) < qddBaseline.get(i, i),
                    "doubling the rotor inertia must lower diag(Qa) for joint " + f.filteredJoints.get(i).getName()
                    + " (was " + qddBaseline.get(i, i) + ", now " + qddStiffer.get(i, i) + ")");
      }

      // Exact reversibility: restoring the defaults must reproduce the baseline to the last bit. Anything that
      // latched, cached or accumulated the parameter shows up here as a non-zero difference.
      for (int i = 0; i < n; i++)
         rotor[i].set(defaults[i]);
      f.filter.predict();
      DMatrixRMaj qddRestored = block(f.filter.getProcessNoise(), n, n, n, n);
      for (int i = 0; i < n; i++)
         for (int j = 0; j < n; j++)
            assertEquals(qddBaseline.get(i, j), qddRestored.get(i, j), 0.0,
                         "restoring the rotor inertia must restore Qa exactly at (" + i + "," + j + ")");
   }

   /**
    * jointKF_QaDiag_&lt;joint&gt; is the measurement the documented ALPHA equalization is calibrated from, so it
    * must track the Qa the filter actually applied on the last predict, not a stale or construction-time value.
    */
   @Test
   public void testQaDiagTelemetryTracksTheAppliedProcessNoise()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairMassMatrix(9200L, 8, 1, 7);
      int n = f.n;
      f.filter.setStateForTest(new DMatrixRMaj(f.dim, 1), JointLevelKFTestFixture.spd(f.dim, 92L));

      double[] q = new double[n];
      for (int i = 0; i < n; i++)
         q[i] = 0.4 - 0.1 * i;
      f.applyConsistentMotion(q, new double[n]);
      f.filter.predict();

      // The qd-qd block is dt * Qa (Van Loan), so diag(Qa)_i = Q[n+i][n+i] / dt.
      DMatrixRMaj qdd = block(f.filter.getProcessNoise(), n, n, n, n);
      for (int i = 0; i < n; i++)
      {
         String jointName = f.filteredJoints.get(i).getName();
         YoDouble yoQaDiag = (YoDouble) f.registry.findVariable("jointKF_QaDiag_" + jointName);
         assertNotNull(yoQaDiag, "jointKF_QaDiag_" + jointName + " must be published");
         assertEquals(qdd.get(i, i) / DT, yoQaDiag.getValue(), 1.0e-9 * Math.max(1.0, Math.abs(yoQaDiag.getValue())),
                      "jointKF_QaDiag_" + jointName + " must equal the applied diag(Qa)");
      }
   }

   /**
    * jointKFParam_qaMax is a TRIPWIRE, not a clamp: crossing it must count and warn but leave Qa untouched.
    * Both halves matter — a live threshold that silently rescaled Q would reintroduce the global-Q-starvation
    * failure mode the tripwire replaced.
    */
   @Test
   public void testQaMaxTripwireIsLiveAndDoesNotRescaleQa()
   {
      JointLevelKFTestFixture f = JointLevelKFTestFixture.singlePairMassMatrix(9300L, 8, 1, 7);
      int n = f.n;
      f.filter.setStateForTest(new DMatrixRMaj(f.dim, 1), JointLevelKFTestFixture.spd(f.dim, 93L));

      double[] q = new double[n];
      for (int i = 0; i < n; i++)
         q[i] = -0.5 + 0.12 * i;
      f.applyConsistentMotion(q, new double[n]);

      YoDouble qaMax = (YoDouble) f.registry.findVariable("jointKFParam_qaMax");
      YoInteger bindCount = (YoInteger) f.registry.findVariable("jointKFQaCapWouldBindCount");
      assertNotNull(qaMax, "jointKFParam_qaMax must be published");
      assertNotNull(bindCount, "jointKFQaCapWouldBindCount must be published");

      // Raise the ceiling far above anything this configuration produces: the tripwire must stay quiet.
      qaMax.set(Double.MAX_VALUE);
      f.filter.predict();
      DMatrixRMaj qddQuiet = block(f.filter.getProcessNoise(), n, n, n, n);
      int countAfterQuiet = bindCount.getValue();
      f.filter.predict();
      assertEquals(countAfterQuiet, bindCount.getValue(), "tripwire must not count while the ceiling is above max diag(Qa)");

      // Drop the ceiling below the observed max: it must start counting on the very next predict...
      double maxQaDiag = 0.0;
      for (int i = 0; i < n; i++)
         maxQaDiag = Math.max(maxQaDiag, qddQuiet.get(i, i) / DT);
      qaMax.set(0.5 * maxQaDiag);
      f.filter.predict();
      assertTrue(bindCount.getValue() > countAfterQuiet, "lowering jointKFParam_qaMax must make the tripwire bind live");

      // ...and Qa must be BIT-IDENTICAL to the un-tripped value. The tripwire surfaces; it never rescales.
      DMatrixRMaj qddTripped = block(f.filter.getProcessNoise(), n, n, n, n);
      for (int i = 0; i < n; i++)
         for (int j = 0; j < n; j++)
            assertEquals(qddQuiet.get(i, j), qddTripped.get(i, j), 0.0,
                         "the QA_MAX tripwire must not rescale Qa at (" + i + "," + j + ")");
   }

   /**
    * The parameter block must be a per-filter instance, not shared state: two filters in one JVM must each own
    * their tuning, or a retune on the dev sim would silently follow into the next pipeline.
    */
   @Test
   public void testParametersArePerFilterInstance()
   {
      JointLevelKFTestFixture a = JointLevelKFTestFixture.singlePairMassMatrix(9400L, 6, 1, 5);
      JointLevelKFTestFixture b = JointLevelKFTestFixture.singlePairMassMatrix(9400L, 6, 1, 5);

      YoDouble qaMaxA = (YoDouble) a.registry.findVariable("jointKFParam_qaMax");
      YoDouble qaMaxB = (YoDouble) b.registry.findVariable("jointKFParam_qaMax");
      assertNotNull(qaMaxA);
      assertNotNull(qaMaxB);
      assertEquals(JointKFParameters.QA_MAX, qaMaxA.getValue(), 0.0, "seeded from the compile-time default");
      assertEquals(JointKFParameters.QA_MAX, qaMaxB.getValue(), 0.0, "seeded from the compile-time default");

      qaMaxA.set(1.0);
      assertEquals(JointKFParameters.QA_MAX, qaMaxB.getValue(), 0.0,
                   "retuning one filter must not touch another filter's parameters");
   }
}
