package us.ihmc.stateEstimation.jointLevel;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import rcl_interfaces.msg.dds.Log;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.sensorProcessing.imu.IMUSensor;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBasedJointStateEstimator;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBiasProvider;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;

/**
 * Joint-level Kalman filter pre-filter (P-A architecture): one filter over the IMU tree whose pair
 * measurements z_w,ab = J(q^) S_ab qd + b_w,ab + v couple the joints on each IMU-pair path, with
 * per-IMU biases in state and a phase-2 gauge anchor for the absolute base bias.
 *
 * <p><b>Do not share an instance between pipelines</b> (the filter holds its covariance).</p>
 * @author Lucas Libshutz
 */
public class JointLevelKFPreFilter implements ProprioceptivePreFilter
{
   //TUNING VARIABLES
   private static final double ENCODER_VAR = 1.0e-6; // (1-e-3 rad)^2 encoder position variance
   private static final double SIGMA_ACCEL = 50.0; // rad/s^2 CWNA process-noise STD
   private static final double INIT_POS_VAR = 1.0e-6; // encoders trusted at initialization
   private static final double INIT_VEL_VAR = 1.0; // velocity unknown at initialization
   private static final double INIT_BIAS_VAR = 2.5e-3; // (0.05 rad/s)^2
   private static final double ANCHOR_VAR = 4.0e-4; // stance FK slip variance

   // Declaration of all pre-allocated variables
   private final SensorOutputMapReadOnly sensorMap;
   private final double dt;

   private final LinkedHashMap<OneDoFJointBasics, Integer> jointToIndex = new LinkedHashMap<>();
   private final LinkedHashMap<IMUSensorReadOnly, Integer> imuToOrdinal = new LinkedHashMap<>();
   private final List<Pair> pairs = new ArrayList<>();
   private final List<FootAnchor> footAnchors = new ArrayList<>();
   private IMUSensorReadOnly baseIMU;

   private int baseBiasCol;
   private int n; // number of distinct joints
   private int m; // number of IMUs
   private int dim; // 2n + 3m
   private boolean initialized = false;

   // Observability: the constructor was previously handed a parentRegistry that it dropped on the floor
   // (so the filter published nothing on hardware). These are wired to the parent registry now.
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoBoolean yoInitialized = new YoBoolean("jointKFInitialized", registry);
   private final YoInteger yoStateDimension = new YoInteger("jointKFStateDimension", registry);
   private final YoInteger yoNumberOfFilteredJoints = new YoInteger("jointKFNumberOfFilteredJoints", registry);
   private final YoInteger yoNumberOfIMUs = new YoInteger("jointKFNumberOfIMUs", registry);

   // State and constant model
   private final DMatrixRMaj x = new DMatrixRMaj(0,1); // reshaped to dimx1 later when constructed
   private final DMatrixRMaj xtmp = new DMatrixRMaj(0,1);
   private final DMatrixRMaj P = new DMatrixRMaj(0,0); // dim x dim
   private final DMatrixRMaj F = new DMatrixRMaj(0,0); // dim x dim transition matrix
   private final DMatrixRMaj Q = new DMatrixRMaj(0,0); // dim x dim process noise
   private DMatrixRMaj Henc, Renc, zEnc;

   // Measurement model matrices
   private final DMatrixRMaj H = new DMatrixRMaj(3,0);
   private final DMatrixRMaj PHt = new DMatrixRMaj(0,0);
   private final DMatrixRMaj S = new DMatrixRMaj(0,0);
   private final DMatrixRMaj Sinv = new DMatrixRMaj(0,0);
   private final DMatrixRMaj K = new DMatrixRMaj(0,0);
   private final DMatrixRMaj nu = new DMatrixRMaj(0,0);
   private final DMatrixRMaj KR = new DMatrixRMaj(0,0); // I-KH, dim x dim, Joseph form
   private final DMatrixRMaj IKH = new DMatrixRMaj(0,0); // I-KH, dim x dim, Joseph form
   private final DMatrixRMaj Ptmp = new DMatrixRMaj(0,0);
   private final DMatrixRMaj zMeas = new DMatrixRMaj(3,1);
   private final DMatrixRMaj zAnchor = new DMatrixRMaj(3,1);
   private final DMatrixRMaj R3 = new DMatrixRMaj(3,3);
   private final DMatrixRMaj Rimu = new DMatrixRMaj(3,3);
   private final DMatrixRMaj rot3 = new DMatrixRMaj(3,3);
   private final DMatrixRMaj tmp3a = new DMatrixRMaj(3,3);
   private final DMatrixRMaj tmp3b = new DMatrixRMaj(3,3);
   private final DMatrixRMaj anchorR = new DMatrixRMaj(3,3);
   // Reused LU solver for the innovation-covariance inverse, pre-sized in allocate(). This replaces the
   // per-tick CommonOps_DDRM.invert(S, Sinv), which allocates a fresh LU decomposition + solver on every
   // call once S is larger than 5x5 (i.e. the n-joint encoder update) — garbage on the estimator thread.
   private LinearSolverDense<DMatrixRMaj> innovationSolver;
   private final RigidBodyTransform tmpTransform = new RigidBodyTransform();
   private final FrameVector3D fvA = new FrameVector3D();
   private final FrameVector3D fvB = new FrameVector3D();
   private final FrameVector3D biasOut = new FrameVector3D();


   // Package-private (not private) so the allocation/behavior tests in this package can build it directly
   // from a synthetic IMU-pair setup without standing up a full StateEstimatorParameters.
   JointLevelKFPreFilter(SensorOutputMapReadOnly sensorMap,
                         List<IMUBasedJointStateEstimatorParameters> pairParameters,
                         Collection<RigidBodyBasics> feet,
                         double estimatorDT,
                         YoRegistry parentRegistry)
   {
      this.sensorMap = sensorMap;
      this.dt = estimatorDT;
      if (parentRegistry != null)
         parentRegistry.addChild(registry);

      // 1)  Resolve all pairs of IMUs, collect distinct joints and IMUs into the state layout
      for (IMUBasedJointStateEstimatorParameters pp : pairParameters)
      {
         IMUSensorReadOnly parent = findIMU(sensorMap, pp.getParentIMUName());
         IMUSensorReadOnly child = findIMU(sensorMap, pp.getChildIMUName());
         if (parent == null || child == null)
         {
            LogTools.warn("Skipping pair, IMU not found: parent = " + pp.getParentIMUName() + " child = " + pp.getChildIMUName());
            continue;
         }
         Pair p = new Pair(parent,child);
         p.jac.setKinematicChain(parent.getMeasurementLink(),child.getMeasurementLink());
         p.jac.setJacobianFrame(child.getMeasurementLink().getBodyFixedFrame());
         //NOTE: Assumes 1-DoF and fixed joints only, just as IMUBasedJointStateEstimator does.
         // The i-th angular jacobian column correspnods to the i-th filtered joint.
         List<OneDoFJointBasics> chain = MultiBodySystemTools.filterJoints(p.jac.getJointsFromBaseToEndEffector(), OneDoFJointBasics.class);
         p.chainJoints = chain.toArray(new OneDoFJointBasics[0]);
         for (OneDoFJointBasics j : p.chainJoints)
            jointToIndex.putIfAbsent(j, jointToIndex.size()); // places joints via hash mapping
         imuToOrdinal.putIfAbsent(parent, imuToOrdinal.size());
         imuToOrdinal.putIfAbsent(child, imuToOrdinal.size());
         pairs.add(p);
      }

      n = jointToIndex.size();
      m = imuToOrdinal.size();
      dim = 2 * n + 3 * m;

      // 2) Second pass: assemble state sinze columns are known as n is fixed.
      for (Pair p: pairs)
      {
         int dof = p.chainJoints.length;
         p.Jang.reshape(3,dof); // Jang is allocated blank at construction in the chain, only reshaped once full DoF are known
         p.qdCols = new int[dof];
         for (int c = 0; c < dof; c++)
            p.qdCols[c] = n + jointToIndex.get(p.chainJoints[c]); // this int[] is the selector matrix of the path S_ab, but in sparse form as all joints are not related directly
         p.parentBias = 2 * n + 3 * imuToOrdinal.get(p.parent);
         p.childBias = 2 * n + 3 * imuToOrdinal.get(p.child);
      }

      // 3) Base IMU = root of the tree + first pair parent.
      //WARNING: Need to configure if the root differs.
      baseIMU = pairs.isEmpty() ? null : pairs.get(0).parent;
      baseBiasCol = baseIMU == null ? -1 : 2 * n + 3 * imuToOrdinal.get(baseIMU);
      //TODO: Log what the base IMU is
      if (baseIMU == null)
         throw new RuntimeException("Base IMU is null, check the kinematic tree.");
      LogTools.info("Base IMU initialized as " + baseIMU.getSensorName());

      // 4) Precompute base->foot chains for the phase 2 stance anchoring measurement update
      if (baseIMU != null && feet != null)
      {
         for (RigidBodyBasics foot : feet)
         {
            FootAnchor fa = new FootAnchor(foot);
            fa.jac.setKinematicChain(baseIMU.getMeasurementLink(), foot);
            fa.jac.setJacobianFrame(baseIMU.getMeasurementFrame()); // express J in the same frame as the base gyro
            List<OneDoFJointBasics> leg = MultiBodySystemTools.filterJoints(fa.jac.getJointsFromBaseToEndEffector(), OneDoFJointBasics.class);

            fa.legJoints = leg.toArray(new OneDoFJointBasics[0]);
            fa.Jang.reshape(3, fa.legJoints.length);
            fa.qdCols = new int[fa.legJoints.length];
            fa.usable = true;
            for (int c = 0; c < fa.legJoints.length; c++)
            {
               Integer idx = jointToIndex.get(fa.legJoints[c]);
               if (idx == null)
               {
                  fa.usable = false;
                  fa.qdCols[c] = -1;
               }
               else
                  fa.qdCols[c] = n + idx;
            }
            footAnchors.add(fa);

         }
      }
      allocate();

      yoStateDimension.set(dim);
      yoNumberOfFilteredJoints.set(n);
      yoNumberOfIMUs.set(m);
   }

   // Mirrors AlphaComplimentaryFilter.createForKinematicsEstiamtor, works for the factory implementation
   public static JointLevelKFPreFilter createForKinematicsEstimator(SensorOutputMapReadOnly sensorOutputMap,
                                                                    StateEstimatorParameters stateEstimatorParameters,
                                                                    List<? extends IMUSensorReadOnly> imuProcessedOutputs,
                                                                    Collection<RigidBodyBasics> feet,
                                                                    double gravitationalAcceleration,
                                                                    BooleanProvider cancelGravityFromAccelerationMeasurement,
                                                                    double estimatorDT,
                                                                    YoRegistry parentRegistry)
   {
      //TODO: this function should be removed and the factory should handle this part.
      if (stateEstimatorParameters == null)
         throw new UnsupportedOperationException("default estimator parameters for this type of estimator are not added yet.");
      // imuProcessedOutputs / gravity / cancelGravity are unused: this filter resolves its IMUs from the
      // sensor map by the pair parameters' names (the same live IMU objects the alpha filter uses) and its
      // stance anchor is gyro-based. They are kept for signature parity with the alpha factory. The
      // registry, however, is now threaded through so the filter actually publishes on hardware.
      return new JointLevelKFPreFilter(sensorOutputMap,
                                       stateEstimatorParameters.getIMUBasedJointStateEstimatorParameters(),
                                       feet,
                                       estimatorDT,
                                       parentRegistry);
   }

   private void allocate()
   {
      int maxMeas = Math.max(n,3); // encoder update is the widest measurement
      x.reshape(dim, 1);
      xtmp.reshape(dim, 1);

      P.reshape(dim,dim);
      F.reshape(dim,dim);
      Q.reshape(dim,dim);
      PHt.reshape(dim,maxMeas);
      K.reshape(dim,maxMeas);
      KR.reshape(dim,maxMeas);
      S.reshape(maxMeas, maxMeas);
      Sinv.reshape(maxMeas,maxMeas);
      nu.reshape(maxMeas,1);
      Henc = new DMatrixRMaj(n,dim);
      Renc = new DMatrixRMaj(n,n);
      zEnc = new DMatrixRMaj(n,1);

      // Pre-size the Joseph-form scratch. IKH in particular MUST start at dim x dim: the first josephUpdate
      // does setIdentity(IKH) *before* multAdd reshapes it, so on a 0x0 IKH the identity is silently dropped
      // and the first covariance update becomes -KH instead of I-KH. Sizing both here also keeps their first
      // use from reallocating on the estimator thread.
      Ptmp.reshape(dim, dim);
      IKH.reshape(dim, dim);

      // Reused LU solver for the innovation-covariance inverse, warmed at the widest measurement (maxMeas).
      // The warm-up forces every internal buffer to its largest size now, so per-tick setA()/invert() reuse
      // them and never allocate — unlike CommonOps_DDRM.invert, which news up a decomposition every call for
      // any matrix wider than 5 (the n-joint encoder update).
      innovationSolver = LinearSolverFactory_DDRM.lu(maxMeas);
      innovationSolver.setA(CommonOps_DDRM.identity(maxMeas));

      for (int i = 0; i < 3; i++)
         anchorR.set(i, i, ANCHOR_VAR);

      buildConstantTransition();
      buildProcessNoise();
      buildEncoderModel();
   }

   private void buildConstantTransition()
   {
      CommonOps_DDRM.setIdentity(F);
      for (int i = 0; i < n; i++)
         F.set(i, n + i, dt); // q_{k+1} = q_k + dt * qd_k ; qd and bias are constant with noise
   }

   private void buildProcessNoise()
   {
      Q.zero();
      double sa2 = SIGMA_ACCEL * SIGMA_ACCEL;
      double dt2 = dt * dt;
      double dt3 = dt2 * dt;

      for (int i = 0; i < n; i++)
      {
         Q.set(i, i, sa2 * dt3/3.0); // CT white noise accel block, per joint
         Q.set(i, n + i, sa2 * dt2 / 2.0); // q - qd cross term
         Q.set(n + i, i, sa2 * dt2 / 2.0);
         Q.set(n + i, n + i, sa2 * dt);
      }
      for (var e : imuToOrdinal.entrySet()) // bias random walk, per-IMU process noise
      {
         e.getKey().getAngularVelocityBiasProcessNoiseCovariance(Rimu);
         int col = 2 * n + 3 * e.getValue();
         for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
               Q.set(col + i, col + j, Q.get(col + i, col + j) + dt * Rimu.get(i,j));
      }
   }

   private void buildEncoderModel()
   {
      Henc.zero();
      for (int i = 0; i < n; i++)
         Henc.set(i, i, 1.0);
      Renc.zero();
      for (int i = 0; i < n; i++)
         Renc.set(i, i, ENCODER_VAR);
   }

   @Override
   public void initialize()
   {
      x.zero();
      for (var e : jointToIndex.entrySet())
         x.set(e.getValue(), sensorMap.getOneDoFJointOutput(e.getKey()).getPosition());
      P.zero();
      for (int i = 0; i < n; i++) P.set(i, i, INIT_POS_VAR);
      for (int i = n; i < 2 * n; i++) P.set(i, i, INIT_VEL_VAR);
      for (int i = 2 * n; i < dim; i++) P.set(i, i, INIT_BIAS_VAR);
      initialized = true;
      yoInitialized.set(true);
   }

   // ================================ Phase 1 ================================

   @Override
   public void computeJointState()
   {
      // Phase 1: joint predict + encoder/pair-gyro measurement updates go here.
      if (!initialized) initialize();

      predict();

      // update encoder states
      int row = 0;
      for (OneDoFJointBasics j : jointToIndex.keySet())
         zEnc.set(row++, 0, sensorMap.getOneDoFJointOutput(j).getPosition());
      josephUpdate(Henc, zEnc, Renc);

      for (int i = 0; i < pairs.size(); i++)
         pairGyroUpdate(pairs.get(i));
   }

   /** EKF time update in isolation: x <- F x, P <- F P F^T + Q. Package-private so tests can drive it alone. */
   void predict()
   {
      CommonOps_DDRM.mult(F, x, xtmp);
      x.set(xtmp);
      CommonOps_DDRM.mult(F, P, Ptmp);
      CommonOps_DDRM.multTransB(Ptmp, F, P); // FPF^T term
      CommonOps_DDRM.addEquals(P, Q);
   }

   private void pairGyroUpdate(Pair p)
   {
      buildPairMeasurement(p);
      josephUpdate(H, zMeas, R3);
   }

   /**
    * Fills the pair-gyro measurement model (H, zMeas, R3) for one IMU pair, without applying the update.
    * Split out of {@link #pairGyroUpdate} so tests can inspect the measurement model (mirrors the reference
    * implementation's build_measurement / build_H seam).
    */
   private void buildPairMeasurement(Pair p)
   {
      // Frames are already current: the estimator calls rootBody.updateFramesRecursively() every tick (this
      // filter runs inside that tick and reads the same joints the alpha estimator does). Re-updating them
      // per joint here is redundant AND allocates ~64 B/joint inside MovingReferenceFrame.update() — garbage
      // on the estimator thread that the alpha filter never generated.
      p.jac.reset();
      CommonOps_DDRM.extract(p.jac.getJacobianMatrix(), 0, 3, 0, p.qdCols.length, p.Jang, 0, 0); //angular part

      // z = omega_child - omega_parent, expressed in the Jacobian frame (same construction as the alpha estimator)
      fvA.setToZero(p.child.getMeasurementFrame());
      fvA.set(p.child.getAngularVelocityMeasurement());
      fvA.changeFrame(p.jac.getJacobianFrame());

      fvB.setToZero(p.parent.getMeasurementFrame());
      fvB.set(p.parent.getAngularVelocityMeasurement());
      fvB.changeFrame(p.jac.getJacobianFrame());
      fvA.sub(fvB);
      fvA.get(zMeas);

      // H (3 x dim) ; velocity block = scattered J columns, bias blocks = +R_child, -R_parent; q block = 0
      H.reshape(3, dim);
      H.zero();
      for (int c = 0; c < p.qdCols.length; c++)
         for (int r = 0; r < 3; r++)
            H.set(r, p.qdCols[c], p.Jang.get(r,c));
      packRotationToJacFrame(p.child, p.jac.getJacobianFrame(), rot3);
      insertScaledInto(rot3, +1.0, H, 0, p.childBias);
      packRotationToJacFrame(p.parent, p.jac.getJacobianFrame(), rot3);
      insertScaledInto(rot3, -1.0, H, 0, p.parentBias);
      // q columns left 0: dJ/dq * qd neglected (encoder has pinned q)
      // R = R_child Sigma_child R_child^T + R_parent Sigma_parent R_parent^T (L Sigma L^T)
      R3.zero();
      packRotationToJacFrame(p.child, p.jac.getJacobianFrame(), rot3);
      p.child.getAngularVelocityBiasProcessNoiseCovariance(Rimu);
      congruenceAdd(rot3, Rimu, R3);
      packRotationToJacFrame(p.parent, p.jac.getJacobianFrame(), rot3);
      p.parent.getAngularVelocityBiasProcessNoiseCovariance(Rimu);
      congruenceAdd(rot3, Rimu, R3);
   }

   // ================================ Phase 2 ================================

   @Override
   public void computeImuBiases(List<RigidBodyBasics> trustedFeet)
   {
      if (!initialized || trustedFeet.isEmpty())
         return; // no absolute reference at the current tick; common mode bias gauge stays free
      for (int i = 0; i < footAnchors.size(); i++)
      {
         FootAnchor fa = footAnchors.get(i);
         if (fa.usable && trustedFeet.contains(fa.foot))
            stanceAnchorUpdate(fa);
      }
   }

   private void stanceAnchorUpdate(FootAnchor fa)
   {
      // Frames already current (see pairGyroUpdate); do not re-update per joint (redundant + allocates).
      CommonOps_DDRM.extract(fa.jac.getJacobianMatrix(), 0, 3, 0, fa.qdCols.length, fa.Jang, 0, 0);

      // Measurement: base IMU gyro, already in measurement frame
      Vector3DReadOnly w = baseIMU.getAngularVelocityMeasurement();
      zAnchor.set(0, 0, w.getX());
      zAnchor.set(1,0, w.getY());
      zAnchor.set(2,0, w.getZ());

      // Model with no contact twist: (omega_foot = 0) => z = -J_ang * qd_leg + b_base
      // Verify with convention before trusting, sign of J_ang and the
      // assumption that the stance foot is non-rotating is what makes this work.
      H.reshape(3,dim);
      H.zero();
      for (int c = 0; c < fa.qdCols.length; c++)
         for (int r = 0; r < 3; r++)
            H.set(r, fa.qdCols[c], -fa.Jang.get(r,c));
      H.set(0, baseBiasCol, 1.0);
      H.set(1, baseBiasCol + 1, 1.0);
      H.set(2, baseBiasCol + 2, 1.0);

      josephUpdate(H, zAnchor, anchorR);
   }

   // ================================ KF core ================================
   // Sequential Joseph form measurement update for any matrix of dimension k.
   // Package-private so tests can drive a single measurement update against a reference KF.
   void josephUpdate(DMatrixRMaj Hm, DMatrixRMaj zm, DMatrixRMaj Rm)
   {
      int k = Hm.getNumRows();
      PHt.reshape(dim, k);
      S.reshape(k, k);
      Sinv.reshape(k,k);
      nu.reshape(k,1);

      CommonOps_DDRM.multTransB(P, Hm, PHt);
      CommonOps_DDRM.mult(Hm, PHt, S);
      CommonOps_DDRM.addEquals(S, Rm); // innovation covariance S = H P H^T + R (the +R was previously missing,
                                       // which made the gain over-confident and could grow the covariance)
      // Allocation-free inverse via the pre-warmed solver (see allocate()). setA decomposes S in place;
      // invert writes S^-1 into Sinv. Neither allocates because S is never wider than the warm-up size.
      if (!innovationSolver.setA(S))
      {
         LogTools.warn("Singular innovation covariance! Skipping update.");
         return;
      }
      innovationSolver.invert(Sinv);
      K.reshape(dim, k);
      CommonOps_DDRM.mult(PHt, Sinv, K);
      CommonOps_DDRM.mult(Hm, x, nu);
      CommonOps_DDRM.changeSign(nu);
      CommonOps_DDRM.addEquals(nu, zm);
      CommonOps_DDRM.multAdd(K, nu, x);

      // Full Joseph form update
      CommonOps_DDRM.setIdentity(IKH);
      CommonOps_DDRM.multAdd(-1.0, K, Hm, IKH); // I - KH
      CommonOps_DDRM.mult(IKH, P, Ptmp);
      CommonOps_DDRM.multTransB(Ptmp, IKH, P);
      KR.reshape(dim, k);
      CommonOps_DDRM.mult(K, Rm, KR);
      CommonOps_DDRM.multAddTransB(KR, K, P); // + K R K^T
   }


   private void packRotationToJacFrame(IMUSensorReadOnly imu, ReferenceFrame jacFrame, DMatrixRMaj out)
   {
      imu.getMeasurementFrame().getTransformToDesiredFrame(tmpTransform, jacFrame);
      RotationMatrixReadOnly r = tmpTransform.getRotation();
      out.reshape(3,3);

      set_matrix(out, r);
   }

   public static void set_matrix(DMatrixRMaj out, RotationMatrixReadOnly r)
   {
      out.set(0, 0, r.getM00());
      out.set(0, 1, r.getM01());
      out.set(0, 2, r.getM02());

      out.set(1, 0, r.getM10());
      out.set(1, 1, r.getM11());
      out.set(1, 2, r.getM12());

      out.set(2, 0, r.getM20());
      out.set(2, 1, r.getM21());
      out.set(2, 2, r.getM22());
   }

   private void congruenceAdd(DMatrixRMaj L, DMatrixRMaj Sigma, DMatrixRMaj accum)
   {
      CommonOps_DDRM.mult(L, Sigma, tmp3a);
      CommonOps_DDRM.multTransB(tmp3a, L, tmp3b);
      CommonOps_DDRM.addEquals(accum, tmp3b);
   }

   private static void insertScaledInto(DMatrixRMaj src, double scale, DMatrixRMaj dst, int row0, int col0)
   {
      for (int i = 0; i < src.getNumRows(); i++)
         for (int j = 0; j < src.getNumCols(); j++)
            dst.set(row0 + i, col0 + j, scale * src.get(i,j));
   }

   private static IMUSensorReadOnly findIMU(SensorOutputMapReadOnly map, String name)
   {
      for (int i = 0; i < map.getIMUOutputs().size(); i++)
      {
         IMUSensorReadOnly s = map.getIMUOutputs().get(i);
         if (s.getSensorName().equals(name)) return s;
      }
      return null;
   }

   @Override
   public boolean containsJoint(OneDoFJointBasics joint)
   {
      return jointToIndex.containsKey(joint);
   }

   @Override
   public double getEstimatedJointPosition(OneDoFJointBasics joint)
   {
      Integer idx = jointToIndex.get(joint);
      return (idx == null || !initialized) ? Double.NaN : x.get(idx);
   }

   @Override
   public double getEstimatedJointVelocity(OneDoFJointBasics joint)
   {
      Integer idx = jointToIndex.get(joint);
      return (idx == null || !initialized) ? Double.NaN : x.get(n + idx);
   }

   @Override
   public boolean hasCovariance()
   {
      return initialized;
   }

   @Override
   public void packPositionCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
   {
      packCov(joints, 0, fallbackVariance, toPack);
   }

   @Override
   public void packVelocityCovariance(OneDoFJointBasics[] joints, double fallbackVariance, DMatrixRMaj toPack)
   {
      packCov(joints, n, fallbackVariance, toPack);
   }

   private void packCov(OneDoFJointBasics[] joints, int blockOffset, double fallbackVariance, DMatrixRMaj toPack)
   {
      int mm = joints.length;
      toPack.reshape(mm, mm);
      toPack.zero();
      for (int a = 0; a < mm; a++)
      {
         Integer ia = jointToIndex.get(joints[a]);
         if (ia == null)
         {
            toPack.set(a, a, fallbackVariance);
            continue;
         }
         for (int b = 0; b < mm; b++)
         {
            Integer ib = jointToIndex.get(joints[b]);
            if (ib != null)
               toPack.set(a, b, P.get(blockOffset + ia, blockOffset + ib)); // adding the off diagonals for the cross-covariances
         }
      }
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      Integer ord = imuToOrdinal.get(imu);
      if (ord == null || !initialized)
      {
         // IMU not in the filter's state (e.g. the primary pelvis IMU when it isn't a pair member),
         // or the filter hasn't initialized yet: report zero bias. MUST return here — falling through
         // would unbox a null ord in "3 * ord" and NPE every tick on the estimator thread.
         biasOut.setToZero(imu.getMeasurementFrame());
         return biasOut;
      }
      int col = 2 * n + 3 * ord;
      biasOut.setIncludingFrame(imu.getMeasurementFrame(), x.get(col), x.get(col + 1), x.get(col + 2));
      return biasOut;
   }

   @Override
   public FrameVector3DReadOnly getAngularVelocityBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      getAngularVelocityBiasInIMUFrame(imu);
      biasOut.changeFrame(ReferenceFrame.getWorldFrame());
      return biasOut;
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInIMUFrame(IMUSensorReadOnly imu)
   {
      biasOut.setToZero(imu.getMeasurementFrame());
      return biasOut;
   }

   @Override
   public FrameVector3DReadOnly getLinearAccelerationBiasInWorldFrame(IMUSensorReadOnly imu)
   {
      biasOut.setToZero(ReferenceFrame.getWorldFrame());
      return biasOut;
   }

   // ================================ Test surface ================================
   // Package-private read/seed hooks used only by the unit tests in this package (mirroring the reference
   // implementation's pure-function seams). Not part of the public API and never called on the estimator
   // thread. Getters return fresh copies so a test cannot mutate filter internals.

   int getStateDimension()          { return dim; }
   int getNumberOfFilteredJoints()  { return n; }
   int getNumberOfIMUs()            { return m; }
   int getNumberOfPairs()           { return pairs.size(); }

   /** State index of the given joint's position entry (its velocity entry is this + n); -1 if not filtered. */
   int getJointStateIndex(OneDoFJointBasics joint) { Integer i = jointToIndex.get(joint); return i == null ? -1 : i; }
   List<OneDoFJointBasics> getFilteredJointsInStateOrder() { return new ArrayList<>(jointToIndex.keySet()); }

   DMatrixRMaj getStateVector()      { return x.copy(); }     // x = [q ; q_dot ; b_omega]
   DMatrixRMaj getCovariance()       { return P.copy(); }
   DMatrixRMaj getTransitionMatrix() { return F.copy(); }
   DMatrixRMaj getProcessNoise()     { return Q.copy(); }
   DMatrixRMaj getEncoderJacobian()  { return Henc.copy(); }
   DMatrixRMaj getEncoderNoise()     { return Renc.copy(); }

   /** Overwrites the mean and covariance (and marks initialized) so tests can drive predict()/josephUpdate() from a known prior. */
   void setStateForTest(DMatrixRMaj xPrior, DMatrixRMaj pPrior)
   {
      x.set(xPrior);
      P.set(pPrior);
      initialized = true;
   }

   /** Builds the pair-gyro measurement (H, z, R) for the given pair without applying the update, for inspection. */
   void buildPairMeasurementForTest(int pairIndex) { buildPairMeasurement(pairs.get(pairIndex)); }
   DMatrixRMaj getMeasurementJacobian() { return H.copy(); }     // last H built (3 x dim)
   DMatrixRMaj getMeasurementResidual() { return zMeas.copy(); } // last z built (3 x 1)
   DMatrixRMaj getMeasurementNoise()    { return R3.copy(); }    // last R built (3 x 3)
   int[] getPairVelocityColumns(int pairIndex) { return pairs.get(pairIndex).qdCols.clone(); }
   int getPairParentBiasColumn(int pairIndex)  { return pairs.get(pairIndex).parentBias; }
   int getPairChildBiasColumn(int pairIndex)   { return pairs.get(pairIndex).childBias; }

   // ================================ Structure holders ================================
   private static final class Pair
   {
      final IMUSensorReadOnly parent, child;
      final GeometricJacobianCalculator jac = new GeometricJacobianCalculator();
      OneDoFJointBasics[] chainJoints;
      int[] qdCols;
      int parentBias, childBias;
      final DMatrixRMaj Jang = new DMatrixRMaj(3,1);
      Pair(IMUSensorReadOnly parent, IMUSensorReadOnly child)
      {
         this.parent = parent;
         this.child = child;
      }
   }

   private static final class FootAnchor
   {
      final RigidBodyBasics foot;
      final GeometricJacobianCalculator jac = new GeometricJacobianCalculator();
      OneDoFJointBasics[] legJoints;
      int[] qdCols;
      boolean usable; // false if any base-> foot joint is not in the state
      final DMatrixRMaj Jang = new DMatrixRMaj(3,1);
      FootAnchor(RigidBodyBasics foot)
      {
         this.foot = foot;
      }
   }

}
