package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

/**
 * The stacked gyro measurement — the only channel that observes the per-IMU gyro biases, and the reason this
 * filter carries them in state at all.
 *
 * <p>ONE Joseph update per tick over rows [0, 3E) for the pairs and [3E, 3E+3K) for this tick's active stance
 * anchors. They share IMU samples, so they MUST NOT be split: a per-block update cannot represent the
 * shared-IMU noise cross-covariance, and a block-diagonal per-pair R double-counts shared samples on a star
 * topology. Biases and gyro white noise both enter through the same edge-incidence operator L, so
 * R_g = L Sigma L^T + Sigma_eps carries those cross-covariances exactly (SPEC §5.3). The bias columns of H_g
 * ARE L — built once per tick and used in both places, so the identity cannot drift.</p>
 *
 * <p>Why the bias needs this channel: the pair rows see the bias only as a DIFFERENCE, leaving the common mode
 * a 3-dim nullspace of H. The anchor's {@code +I3} on the base-IMU bias column is the one absolute bias
 * observation in the filter, and hence the only thing fixing that gauge. With no active anchor the base gyro
 * bias random-walks straight into the downstream InEKF's orientation (FINDINGS.md Part F) — watch
 * {@code jointKFActiveAnchorCount} and {@code jointKF_biasPTrace}, both published here.</p>
 *
 * @author Lucas Libshutz
 */
final class JointKFBiasUpdate
{
   private final JointKFState state;
   private final JointKFUpdate update;
   private final JointKFParameters parameters;

   DMatrixRMaj Hg;    // 3(E+K) x dim measurement Jacobian [ 0 | J_stack(q^) | L(q^) ]
   DMatrixRMaj zg;    // 3(E+K) x 1 stacked measurement (raw gyro samples)
   DMatrixRMaj Rg;    // 3(E+K) x 3(E+K) stacked measurement noise L Sigma L^T + Sigma_eps
   DMatrixRMaj Lmix;  // 3(E+K) x 3m mixing operator L (also copied into H_g's bias columns)
   private DMatrixRMaj Sigma;  // 3m x 3m blkdiag of each IMU's angular-velocity MEASUREMENT-noise covariance
   private DMatrixRMaj LSigma; // 3(E+K) x 3m scratch for the L*Sigma product
   /** Constant I3 for the anchor L-blocks; its setIdentity is in this constructor, next to the declaration. A
    *  zeroed identity3 silently deletes the anchor's +I3 and un-fixes the gauge — no error, no NaN. */
   private final DMatrixRMaj identity3 = new DMatrixRMaj(3, 3);
   /** 3x3 scratch for the gyro MEASUREMENT-noise covariance. Deliberately NOT shared with JointKFPrediction's
    *  bias PROCESS-noise scratch — see that field. */
   private final DMatrixRMaj Rimu = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj rot3 = new DMatrixRMaj(3, 3);
   private final RigidBodyTransform tmpTransform = new RigidBodyTransform();
   private final FrameVector3D fvA = new FrameVector3D();
   private final FrameVector3D fvB = new FrameVector3D();

   /** The IHMC estimator finalizes contact trust in phase 2, AFTER computeJointState, so this phase-1 update
    *  must use the previous tick's set. Pre-sized to the anchor count; only cleared/refilled, never grown. */
   private final List<RigidBodyBasics> trustedFeetFromLastTick;

   private boolean sigmaFloorInitialized = false; // Sigma validated/floored/cached on first buildStackedMeasurement
   private boolean gyroSigmaFloored = false;      // any IMU's gyro noise was floored
   private boolean warnedGyroFloorRegression = false; // one-shot: R_g pair-row diagonal fell below the floor at runtime

   // The two variables that would have caught the pitch drift months earlier, and that condS/minSDiag CANNOT: a
   // nullspace of H is invisible to S by construction (the measured bias-DIFFERENCE rows stay well-conditioned
   // even while the common mode is unfixed) and shows up only in P. Anchor count 0 => gauge unfixed => the bias
   // WILL diverge; an unboundedly growing trace says the same. See FINDINGS.md §F.4.
   private final YoInteger yoActiveAnchorCount;
   private final YoDouble yoBiasPTrace;
   // Per-IMU gyro bias (IMU frame), indexed by ordinal. Published because this is the bias EXPORTED to the
   // downstream InEKF, where a runaway integrates straight into base orientation — not an unlogged black box.
   private final YoDouble[] yoImuGyroBiasX;
   private final YoDouble[] yoImuGyroBiasY;
   private final YoDouble[] yoImuGyroBiasZ;
   private final YoDouble[] yoImuGyroBiasNorm;

   JointKFBiasUpdate(JointKFState state, JointKFUpdate update, JointKFParameters parameters, YoRegistry registry)
   {
      this.state = state;
      this.update = update;
      this.parameters = parameters;

      int m = state.numberOfIMUs;
      int dim = state.dim;
      int maxStackRows = state.maxStackRows;

      // Pre-sized at K = K_max active anchors; each tick reshapes DOWN within this capacity, so no per-tick
      // allocation. Lmix/Sigma/LSigma feed the R_g congruence and H_g's bias columns.
      Hg = new DMatrixRMaj(maxStackRows, dim);
      zg = new DMatrixRMaj(maxStackRows, 1);
      Rg = new DMatrixRMaj(maxStackRows, maxStackRows);
      Lmix = new DMatrixRMaj(maxStackRows, 3 * m);
      Sigma = new DMatrixRMaj(3 * m, 3 * m);
      LSigma = new DMatrixRMaj(maxStackRows, 3 * m);
      CommonOps_DDRM.setIdentity(identity3);
      trustedFeetFromLastTick = new ArrayList<>(Math.max(1, state.footAnchors.size()));

      // Sigma is floored and cached on the FIRST buildStackedMeasurement, not here — see buildAndFloorSigma.

      yoActiveAnchorCount = new YoInteger("jointKFActiveAnchorCount", registry);
      yoBiasPTrace = new YoDouble("jointKF_biasPTrace", registry);
      yoImuGyroBiasX = new YoDouble[m];
      yoImuGyroBiasY = new YoDouble[m];
      yoImuGyroBiasZ = new YoDouble[m];
      yoImuGyroBiasNorm = new YoDouble[m];
      for (int o = 0; o < m; o++)
      {
         String imuName = state.imusByOrdinal[o].getSensorName();
         yoImuGyroBiasX[o] = new YoDouble("jointKF_gyroBias_" + imuName + "_X", registry);
         yoImuGyroBiasY[o] = new YoDouble("jointKF_gyroBias_" + imuName + "_Y", registry);
         yoImuGyroBiasZ[o] = new YoDouble("jointKF_gyroBias_" + imuName + "_Z", registry);
         yoImuGyroBiasNorm[o] = new YoDouble("jointKF_gyroBias_" + imuName + "_norm", registry);
      }
   }

   /**
    * Caches THIS tick's trusted feet for next tick's anchor rows — all of phase 2 now. Do NOT run a measurement
    * update here: splitting the anchors back out would make the base-gyro correlation between them and the
    * base-adjacent pairs unexpressible, which is exactly the bug this revision removed. Refilled with an index
    * loop, not addAll (which allocates via Collection.toArray), so this stays allocation-free.
    */
   void cacheTrustedFeet(List<RigidBodyBasics> trustedFeet)
   {
      trustedFeetFromLastTick.clear();
      if (trustedFeet != null)
      {
         for (int i = 0; i < trustedFeet.size(); i++)
            trustedFeetFromLastTick.add(trustedFeet.get(i));
      }
   }

   /** Anchors active on the last {@link #buildStackedMeasurement}. 0 => the base-bias gauge is unfixed. */
   int getActiveAnchorCount()
   {
      return yoActiveAnchorCount.getValue();
   }

   /**
    * Builds the stacked block and applies it as ONE Joseph update through the shared KF core. If ANY entry is
    * non-finite the WHOLE block is skipped — never individual rows, since a partial stack silently changes the
    * bias-gauge structure.
    */
   void applyStackedUpdate()
   {
      buildStackedMeasurement();
      if (JointLevelKFPreFilter.containsNonFinite(zg)
          || JointLevelKFPreFilter.containsNonFinite(Hg)
          || JointLevelKFPreFilter.containsNonFinite(Rg))
      {
         if (!state.warnedNonFiniteInput)
            state.warnNonFiniteInputOnce("stacked gyro measurement (pairs/anchors)");
         return;
      }
      update.josephUpdate(Hg, zg, Rg, JointKFUpdate.Channel.STACKED_GYRO);
      state.warnIfNonFiniteState("stackedGyroUpdate", -1);
   }

   /**
    * Assembles H_g, z_g, R_g and L for this tick WITHOUT applying the update. Frames are already current (the
    * estimator calls {@code updateFramesRecursively()} every tick) so they are deliberately not re-updated here.
    * Allocation-free: every matrix is pre-sized and only reshaped DOWN.
    *
    * <p>Convention traps, each of which fails SILENTLY if wrong, and all pinned by the oracle test: the pair
    * residual is child-plus / parent-minus, matching {@code setKinematicChain(parent, child)}; L's pair blocks
    * carry the same signs; the anchor gets -J_leg on its q̇-columns and +I3 on the base IMU's bias column
    * (identity because J's frame IS the base measurement frame); and Sigma is the gyro MEASUREMENT-noise
    * covariance, never the bias process noise.</p>
    */
   void buildStackedMeasurement()
   {
      int n = state.numberOfJoints;
      int m = state.numberOfIMUs;
      int dim = state.dim;
      int E = state.numberOfIMUPairs;

      // One-time; the warn/error strings build only on this first fire, and the cached Sigma is reused unchanged.
      if (!sigmaFloorInitialized)
      {
         buildAndFloorSigma();
         sigmaFloorInitialized = true;
      }

      // Active = usable AND trusted last tick. activeAnchors == 0 means the bias gauge is unfixed THIS TICK.
      int activeAnchors = 0;
      for (int i = 0; i < state.footAnchors.size(); i++)
      {
         JointKFState.FootAnchor fa = state.footAnchors.get(i);
         fa.active = fa.usable && trustedFeetFromLastTick.contains(fa.foot);
         if (fa.active)
            activeAnchors++;
      }
      yoActiveAnchorCount.set(activeAnchors);
      int rows = 3 * (E + activeAnchors);

      Hg.reshape(rows, dim);       Hg.zero();
      zg.reshape(rows, 1);         zg.zero();
      Lmix.reshape(rows, 3 * m);   Lmix.zero();
      Rg.reshape(rows, rows);      Rg.zero();

      // Sigma is NOT rebuilt here: re-reading getAngularVelocityNoiseCovariance every tick would re-admit a
      // possibly-zero covariance and reopen the min-side S collapse.

      // Pair rows.
      for (int e = 0; e < E; e++)
      {
         JointKFState.Pair p = state.pairs.get(e);
         int row0 = 3 * e;
         p.jac.reset();
         CommonOps_DDRM.extract(p.jac.getJacobianMatrix(), 0, 3, 0, p.qdCols.length, p.Jang, 0, 0); // angular part

         // z_e = omega_child - omega_parent expressed in the child body-fixed Jacobian frame (child +, parent -).
         fvA.setToZero(p.child.getMeasurementFrame());
         fvA.set(p.child.getAngularVelocityMeasurement());
         fvA.changeFrame(p.jac.getJacobianFrame());
         fvB.setToZero(p.parent.getMeasurementFrame());
         fvB.set(p.parent.getAngularVelocityMeasurement());
         fvB.changeFrame(p.jac.getJacobianFrame());
         fvA.sub(fvB);
         zg.set(row0, 0, fvA.getX());
         zg.set(row0 + 1, 0, fvA.getY());
         zg.set(row0 + 2, 0, fvA.getZ());

         // H_g q̇-columns: +J_e scattered onto the path joints. q-columns left 0 (SPEC §5.4); bias columns via L.
         for (int c = 0; c < p.qdCols.length; c++)
            for (int r = 0; r < 3; r++)
               Hg.set(row0 + r, p.qdCols[c], p.Jang.get(r, c));

         // L pair blocks: +R(child->J_e) at the child IMU bias column, -R(parent->J_e) at the parent IMU column.
         // Lmix column = state bias column minus the 2n (q,q̇) offset (p.childBias/parentBias were precomputed).
         packRotationToJacFrame(p.child, p.jac.getJacobianFrame(), rot3);
         insertScaledInto(rot3, +1.0, Lmix, row0, p.childBias - 2 * n);
         packRotationToJacFrame(p.parent, p.jac.getJacobianFrame(), rot3);
         insertScaledInto(rot3, -1.0, Lmix, row0, p.parentBias - 2 * n);
      }

      // Active stance-anchor rows. A planted foot has zero angular velocity, so
      //    omega_base^meas = -J_F qd_F - J_U qd_U + b_base + noise
      // over filtered F and unfiltered U leg joints (U = the ankles on Alex, which have no foot IMU and so never
      // became states). The unfiltered ones are KNOWN, not estimated, so they move to the measurement side:
      //    z' = omega_base^meas + J_U qd_U^meas = -J_F qd_F + b_base + noise'
      // H gets -J_F on the FILTERED columns only; the L block stays +I3 on the base IMU. That +I3 does not
      // depend on which leg joints are states, so the anchor's gauge-fixing role survives the split intact. The
      // price is qd_U^meas's encoder noise, propagated into R below.
      int arow = 3 * E;
      for (int i = 0; i < state.footAnchors.size(); i++)
      {
         JointKFState.FootAnchor fa = state.footAnchors.get(i);
         if (!fa.active)
            continue;
         fa.jac.reset();
         CommonOps_DDRM.extract(fa.jac.getJacobianMatrix(), 0, 3, 0, fa.qdCols.length, fa.Jang, 0, 0);

         Vector3DReadOnly w = state.baseIMU.getAngularVelocityMeasurement();
         zg.set(arow, 0, w.getX());
         zg.set(arow + 1, 0, w.getY());
         zg.set(arow + 2, 0, w.getZ());

         // Sigma_eps + J_U diag(sigma_qd^2) J_U^T, accumulated as we walk the chain.
         fa.R.zero();
         double anchorVar = parameters.anchorVar.getValue();
         for (int r = 0; r < 3; r++)
            fa.R.add(r, r, anchorVar);

         for (int c = 0; c < fa.qdCols.length; c++)
         {
            if (fa.qdCols[c] >= 0)
            {
               // Filtered joint: a state, so it gets an H column.
               for (int r = 0; r < 3; r++)
                  Hg.set(arow + r, fa.qdCols[c], -fa.Jang.get(r, c)); // -J_F
            }
            else
            {
               // Unfiltered joint: fold J_U(:,c) * qd^meas into z', and its noise into R.
               double qdMeasured = state.sensorMap.getOneDoFJointOutput(fa.legJoints[c]).getVelocity();
               if (!Double.isFinite(qdMeasured))
               {
                  state.warnNonFiniteInputOnce("joint velocity of unfiltered anchor joint " + fa.legJoints[c].getName());
                  qdMeasured = 0.0;
               }
               for (int r = 0; r < 3; r++)
                  zg.add(arow + r, 0, fa.Jang.get(r, c) * qdMeasured); // z' = z + J_U qd_U^meas

               // Rank-1 congruence: J_U(:,c) sigma_c^2 J_U(:,c)^T, per-joint sigma (measured; fallback otherwise).
               for (int r = 0; r < 3; r++)
                  for (int cc = 0; cc < 3; cc++)
                     fa.R.add(r, cc, fa.qdVar[c] * fa.Jang.get(r, c) * fa.Jang.get(cc, c));
            }
         }

         insertScaledInto(identity3, +1.0, Lmix, arow, state.baseBiasCol - 2 * n); // +I3, base measurement frame
         arow += 3;
      }

      // Copy L into H_g's bias columns [2n, 2n+3m): "the bias columns of H_g ARE L" (SPEC §5.3), one build, two uses.
      for (int r = 0; r < rows; r++)
         for (int c = 0; c < 3 * m; c++)
            Hg.set(r, 2 * n + c, Lmix.get(r, c));

      // R_g = L Sigma L^T + the per-anchor Sigma_eps block. The congruence reproduces the SPEC §5.3 block table
      // exactly. Each anchor's own 3x3 is configuration-dependent (built in the loop above), which is desirable
      // and not merely correct: the anchor de-weights itself in postures where the unfiltered ankle Jacobian is
      // large, i.e. exactly where an ankle-velocity error would do the most damage to the base bias.
      LSigma.reshape(rows, 3 * m);
      CommonOps_DDRM.mult(Lmix, Sigma, LSigma);
      CommonOps_DDRM.multTransB(LSigma, Lmix, Rg);
      int arow2 = 3 * E;
      for (int i = 0; i < state.footAnchors.size(); i++)
      {
         JointKFState.FootAnchor fa = state.footAnchors.get(i);
         if (!fa.active)
            continue;
         for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
               Rg.add(arow2 + r, arow2 + c, fa.R.get(r, c));
         arow2 += 3;
      }
      // Exact symmetry for the Joseph update (the congruence is symmetric only to round-off).
      JointLevelKFPreFilter.symmetrize(Rg);

      // Each pair row of L Sigma L^T is a rotation-congruence of the floored parent+child blocks, so its
      // diagonal is ~2 * floor and cannot drop below the floor unless the L/Sigma assembly has regressed.
      if (!warnedGyroFloorRegression)
      {
         double minPairDiag = Double.POSITIVE_INFINITY;
         for (int r = 0; r < 3 * E; r++)
            minPairDiag = Math.min(minPairDiag, Rg.get(r, r));
         double gyroFloor = parameters.sigmaGyroFloor.getValue();
         if (minPairDiag < 0.5 * gyroFloor)
         {
            warnedGyroFloorRegression = true;
            LogTools.warn("JointLevelKFPreFilter: min R_g pair-row diagonal " + minPairDiag + " fell below half the gyro floor "
                  + gyroFloor + " — the Sigma floor should make this impossible; the stacked-measurement assembly (L or "
                  + "Sigma) has regressed. Reported once.");
         }
      }
   }

   /**
    * Validates and floors each IMU's gyro MEASUREMENT-noise covariance ONCE, filling the cached block-diagonal
    * {@code Sigma}. A zero / non-finite trace (an unset SensorNoiseParameters — Alex historically ran with a
    * null one) is an ERROR, because a zero Sigma removes the innovation floor on the pure-bias rows of every
    * 1-DoF chain, collapsing lambda_min(S) and diverging P; a positive-but-low trace is a softer WARN.
    *
    * <p><b>Called lazily on the FIRST {@link #buildStackedMeasurement()}, not at construction</b>, and that
    * deferral is load-bearing: an IMU's SensorNoiseParameters may be assigned AFTER this filter is built — the
    * package tests do exactly that — so reading at construction would see all-zero covariances, floor every
    * IMU, and change R_g. Do not hoist this call. (An older javadoc claimed "at construction" and was wrong;
    * AlexEstimatorLogReplay repeats that error.)</p>
    */
   private void buildAndFloorSigma()
   {
      double gyroFloor = parameters.sigmaGyroFloor.getValue();
      double gyroFloorTrace = parameters.sigmaGyroFloorTrace.getValue();
      Sigma.zero();
      for (int o = 0; o < state.numberOfIMUs; o++)
      {
         state.imusByOrdinal[o].getAngularVelocityNoiseCovariance(Rimu);
         boolean nonFinite = JointLevelKFPreFilter.containsNonFinite(Rimu);
         double trace = nonFinite ? Double.NaN : Rimu.get(0, 0) + Rimu.get(1, 1) + Rimu.get(2, 2);
         if (nonFinite || !(trace >= gyroFloorTrace))
         {
            gyroSigmaFloored = true;
            String name = state.imusByOrdinal[o].getSensorName();
            if (nonFinite || !(trace > 0.0))
               LogTools.error("JointLevelKFPreFilter: IMU '" + name + "' has an unset/zero/non-finite angular-velocity "
                     + "noise covariance (trace=" + trace + "). A zero Sigma removes the innovation-covariance floor on "
                     + "the pure-bias rows of every 1-DoF chain (the min-side S collapse). Substituting the gyro floor "
                     + gyroFloor + " (rad/s)^2 * I3; fix the sensor SensorNoiseParameters at the source.");
            else
               LogTools.warn("JointLevelKFPreFilter: IMU '" + name + "' gyro-noise trace " + trace + " (rad/s)^2 is below "
                     + "the conditioning floor " + gyroFloorTrace + "; substituting " + gyroFloor
                     + " * I3 for innovation-covariance conditioning.");
            Rimu.zero();
            for (int d = 0; d < 3; d++)
               Rimu.set(d, d, gyroFloor);
         }
         insertScaledInto(Rimu, 1.0, Sigma, 3 * o, 3 * o);
      }
   }

   /**
    * Publishes the per-IMU gyro bias (IMU frame) and the trace of P's base-bias block. A physical MEMS gyro bias
    * is ~0.001-0.01 rad/s, so a norm approaching 0.1+ rad/s is a divergence. The trace detects an unfixed gauge:
    * an unobservable direction has no measurement to shrink it, so bounded => the anchor is working, while
    * monotonically climbing => the base bias is unobservable and will diverge.
    */
   void updateBiasYoVariables()
   {
      int n = state.numberOfJoints;
      for (int o = 0; o < state.numberOfIMUs; o++)
      {
         int col = 2 * n + 3 * o;
         double bx = state.x.get(col), by = state.x.get(col + 1), bz = state.x.get(col + 2);
         yoImuGyroBiasX[o].set(bx);
         yoImuGyroBiasY[o].set(by);
         yoImuGyroBiasZ[o].set(bz);
         yoImuGyroBiasNorm[o].set(Math.sqrt(bx * bx + by * by + bz * bz));
      }

      if (state.baseBiasCol >= 0)
         yoBiasPTrace.set(state.P.get(state.baseBiasCol, state.baseBiasCol)
                          + state.P.get(state.baseBiasCol + 1, state.baseBiasCol + 1)
                          + state.P.get(state.baseBiasCol + 2, state.baseBiasCol + 2));
   }

   /** Overwrites the cached previous-tick trusted-feet set that {@link #buildStackedMeasurement} reads anchors from. */
   void setTrustedFeetForTest(List<RigidBodyBasics> feet)
   {
      cacheTrustedFeet(feet);
   }

   private void packRotationToJacFrame(IMUSensorReadOnly imu, ReferenceFrame jacFrame, DMatrixRMaj out)
   {
      imu.getMeasurementFrame().getTransformToDesiredFrame(tmpTransform, jacFrame);
      RotationMatrixReadOnly r = tmpTransform.getRotation();
      out.reshape(3, 3);
      JointLevelKFPreFilter.set_matrix(out, r);
   }

   private static void insertScaledInto(DMatrixRMaj src, double scale, DMatrixRMaj dst, int row0, int col0)
   {
      for (int i = 0; i < src.getNumRows(); i++)
         for (int j = 0; j < src.getNumCols(); j++)
            dst.set(row0 + i, col0 + j, scale * src.get(i, j));
   }
}
