package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.function.ToDoubleFunction;

import org.ejml.data.Complex_F64;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Shared harness for the {@link JointLevelKFPreFilter} unit tests, ported from the JAX reference in
 * {@code invariant-estimation/jointKF}. It stands up a real (but synthetic) IMU-pair setup so the tests can
 * exercise the actual filter math — a random floating revolute chain, hand-built {@link IMUSensorReadOnly}s
 * with controllable measurements + positive-definite covariances, and a mutable sensor map for encoders.
 *
 * <p>Note the state layout the reference calls {@code x = [q ; q_dot ; b_omega] ∈ R^{2n+3m}} carries over,
 * but here {@code m = number of distinct IMUs} (the bias is per-IMU), not the reference's per-pair bias, and
 * the filter always has at least one pair (the "encoder-only" {@code m = 0} case cannot be constructed).</p>
 */
final class JointLevelKFTestFixture
{
   static final double DT = 1.0e-3;
   static final double IMU_BIAS_PROCESS_VAR = 1.0e-4;

   final JointLevelKFPreFilter filter;
   /** Parent of the filter's registry, so tests can findVariable(...) the filter's published YoVariables. */
   final YoRegistry registry;
   final List<RevoluteJoint> joints;
   final List<OneDoFJointBasics> filteredJoints; // in filter state order
   final List<TestIMU> imus;
   final TestSensorMap sensorMap;
   final List<RigidBodyBasics> feet;
   final RigidBodyBasics elevator;      // root of the frame tree; updateFramesRecursively() after setting joint state
   final FloatingJointBasics rootJoint; // floating base; its twist is zeroed so link gyros come only from the joints
   final int n;
   final int m;
   final int dim;

   private final Twist twistTemp = new Twist(); // scratch for reading a link's body-frame angular velocity

   private JointLevelKFTestFixture(JointLevelKFPreFilter filter,
                                   YoRegistry registry,
                                   List<RevoluteJoint> joints,
                                   List<TestIMU> imus,
                                   TestSensorMap sensorMap,
                                   List<RigidBodyBasics> feet,
                                   RigidBodyBasics elevator,
                                   FloatingJointBasics rootJoint)
   {
      this.filter = filter;
      this.registry = registry;
      this.joints = joints;
      this.imus = imus;
      this.sensorMap = sensorMap;
      this.feet = feet;
      this.elevator = elevator;
      this.rootJoint = rootJoint;
      this.filteredJoints = filter.getFilteredJointsInStateOrder();
      this.n = filter.getNumberOfFilteredJoints();
      this.m = filter.getNumberOfIMUs();
      this.dim = filter.getStateDimension();
   }

   /**
    * A spread of pair configurations, the Java analogue of the reference's {@code SHAPES} parametrization.
    * Each yields a different (n, m): three single-pair spans plus one two-pair (3-IMU) layout. The reference's
    * encoder-only {@code m = 0} case is intentionally absent — this filter requires at least one IMU pair.
    */
   static List<JointLevelKFTestFixture> shapes(long baseSeed)
   {
      List<JointLevelKFTestFixture> list = new ArrayList<>();
      list.add(singlePair(baseSeed + 1, 10, 1, 9)); // n = 8, m = 2, 1 pair
      list.add(singlePair(baseSeed + 2, 6, 1, 5));  // n = 4, m = 2, 1 pair
      list.add(singlePair(baseSeed + 3, 4, 0, 3));  // n = 3, m = 2, 1 pair
      list.add(twoPairs(baseSeed + 4, 10, 1, 5, 9)); // n = 8, m = 3, 2 pairs
      return list;
   }

   String describe()
   {
      return "[n=" + n + ", m=" + m + ", pairs=" + filter.getNumberOfPairs() + "]";
   }

   /** Same shape spread as {@link #shapes} but with the chain's elevator handed to the filter, so the
    *  mass-matrix process-noise path (Qa = σ_τ² M(q)⁻²) is active instead of the scalar-CWNA fallback. */
   static List<JointLevelKFTestFixture> shapesMassMatrix(long baseSeed)
   {
      List<JointLevelKFTestFixture> list = new ArrayList<>();
      list.add(singlePairMassMatrix(baseSeed + 1, 10, 1, 9));
      list.add(singlePairMassMatrix(baseSeed + 2, 6, 1, 5));
      list.add(singlePairMassMatrix(baseSeed + 3, 4, 0, 3));
      list.add(build(baseSeed + 4, 10, new int[] {1, 5, 9}, new String[] {"imuA", "imuB", "imuC"},
                     new int[][] {{0, 1}, {1, 2}}, 2, -1, true)); // two pairs, 3 IMUs
      return list;
   }

   /** One IMU pair: parent IMU on body after {@code parentJointIndex}, child on body after {@code childJointIndex}. */
   static JointLevelKFTestFixture singlePair(long seed, int numJoints, int parentJointIndex, int childJointIndex)
   {
      return build(seed,
                   numJoints,
                   new int[] {parentJointIndex, childJointIndex},
                   new String[] {"imuA", "imuB"},
                   new int[][] {{0, 1}},
                   1, // foot = child link
                   -1, // no poisoned IMU
                   false); // scalar-CWNA process noise (no robot model)
   }

   /**
    * Like {@link #singlePair} but with a per-joint encoder position measurement-noise STD lookup (by joint
    * name; NaN / non-positive falls back to the filter's built-in scalar), for the per-joint R wiring and
    * NIS-consistency tests.
    */
   static JointLevelKFTestFixture singlePairWithEncoderNoise(long seed,
                                                             int numJoints,
                                                             int parentJointIndex,
                                                             int childJointIndex,
                                                             ToDoubleFunction<String> encoderPositionNoiseStd)
   {
      return build(seed,
                   numJoints,
                   new int[] {parentJointIndex, childJointIndex},
                   new String[] {"imuA", "imuB"},
                   new int[][] {{0, 1}},
                   1,
                   -1,
                   false,
                   -1,
                   encoderPositionNoiseStd,
                   null,
                   null,
                   false);
   }

   /**
    * Like {@link #singlePairWithEncoderNoise} but with the direct joint-velocity measurement channel enabled:
    * per-joint velocity noise STD and effective low-pass corner (Hz) lookups, both by joint name (corner null
    * or NaN => no lag inflation).
    */
   static JointLevelKFTestFixture singlePairWithDirectVelocity(long seed,
                                                               int numJoints,
                                                               int parentJointIndex,
                                                               int childJointIndex,
                                                               ToDoubleFunction<String> encoderPositionNoiseStd,
                                                               ToDoubleFunction<String> encoderVelocityNoiseStd,
                                                               ToDoubleFunction<String> velocityBreakFrequencyHz)
   {
      return build(seed,
                   numJoints,
                   new int[] {parentJointIndex, childJointIndex},
                   new String[] {"imuA", "imuB"},
                   new int[][] {{0, 1}},
                   1,
                   -1,
                   false,
                   -1,
                   encoderPositionNoiseStd,
                   encoderVelocityNoiseStd,
                   velocityBreakFrequencyHz,
                   true);
   }

   /** Like {@link #singlePair} but with the mass-matrix process-noise path enabled (elevator handed to the filter). */
   static JointLevelKFTestFixture singlePairMassMatrix(long seed, int numJoints, int parentJointIndex, int childJointIndex)
   {
      return build(seed,
                   numJoints,
                   new int[] {parentJointIndex, childJointIndex},
                   new String[] {"imuA", "imuB"},
                   new int[][] {{0, 1}},
                   1,
                   -1,
                   true);
   }

   /**
    * Like {@link #singlePair} but one IMU's angular-velocity bias process-noise covariance is set non-finite
    * <em>before</em> the filter is constructed, so the construction-time guard in {@code buildProcessNoise} /
    * {@code validateConstantModel} is exercised (regression for the NaN hardening).
    */
   static JointLevelKFTestFixture singlePairPoisonBias(long seed, int numJoints, int parentJointIndex, int childJointIndex, int poisonImuIndex)
   {
      return build(seed,
                   numJoints,
                   new int[] {parentJointIndex, childJointIndex},
                   new String[] {"imuA", "imuB"},
                   new int[][] {{0, 1}},
                   1,
                   poisonImuIndex,
                   false);
   }

   /** Two pairs sharing a middle IMU: (a,b) and (b,c) → 3 IMUs, foot on the far (c) link. */
   static JointLevelKFTestFixture twoPairs(long seed, int numJoints, int aIndex, int bIndex, int cIndex)
   {
      return build(seed,
                   numJoints,
                   new int[] {aIndex, bIndex, cIndex},
                   new String[] {"imuA", "imuB", "imuC"},
                   new int[][] {{0, 1}, {1, 2}},
                   2, // foot = c link
                   -1,
                   false);
   }

   /**
    * ALEX'S ACTUAL TOPOLOGY: the foot sits BEYOND the last IMU, so the joints between the last IMU's link and the
    * foot (Alex: the ankles) are on the base→foot chain but are NOT filter states — because with no foot IMU
    * there is no shin→foot pair to bring them in. One IMU pair spanning joints (parent..child], foot on the
    * successor of {@code footJointIndex} further down the chain.
    *
    * <p>This is the configuration whose anchor used to be silently disabled, taking the base gyro-bias gauge
    * with it. See FINDINGS.md Part F.
    */
   static JointLevelKFTestFixture singlePairFootBeyondIMUs(long seed, int numJoints, int parentJointIndex, int childJointIndex, int footJointIndex)
   {
      return build(seed,
                   numJoints,
                   new int[] {parentJointIndex, childJointIndex},
                   new String[] {"imuA", "imuB"},
                   new int[][] {{0, 1}},
                   1,
                   -1,
                   false,
                   footJointIndex);
   }

   private static JointLevelKFTestFixture build(long seed,
                                                int numJoints,
                                                int[] imuBodyJointIndex,
                                                String[] imuNames,
                                                int[][] pairParentChild,
                                                int footImuIndex,
                                                int poisonImuIndex,
                                                boolean useMassMatrixProcessNoise)
   {
      return build(seed, numJoints, imuBodyJointIndex, imuNames, pairParentChild, footImuIndex, poisonImuIndex, useMassMatrixProcessNoise, -1);
   }

   /** @param footJointIndex absolute joint index whose successor is the foot, or -1 to use {@code footImuIndex}'s link. */
   private static JointLevelKFTestFixture build(long seed,
                                                int numJoints,
                                                int[] imuBodyJointIndex,
                                                String[] imuNames,
                                                int[][] pairParentChild,
                                                int footImuIndex,
                                                int poisonImuIndex,
                                                boolean useMassMatrixProcessNoise,
                                                int footJointIndex)
   {
      return build(seed, numJoints, imuBodyJointIndex, imuNames, pairParentChild, footImuIndex, poisonImuIndex, useMassMatrixProcessNoise, footJointIndex,
                   null, null, null, false);
   }

   /**
    * @param encoderPositionNoiseStd per-joint encoder position noise STD lookup handed to the filter, or null for the scalar default.
    * @param encoderVelocityNoiseStd per-joint encoder velocity noise STD lookup, or null for the scalar default.
    * @param velocityBreakFrequencyHz per-joint effective low-pass corner of the velocity measurement, or null for no lag inflation.
    * @param useDirectVelocityMeasurement boot-time enable for the direct-velocity channel.
    */
   private static JointLevelKFTestFixture build(long seed,
                                                int numJoints,
                                                int[] imuBodyJointIndex,
                                                String[] imuNames,
                                                int[][] pairParentChild,
                                                int footImuIndex,
                                                int poisonImuIndex,
                                                boolean useMassMatrixProcessNoise,
                                                int footJointIndex,
                                                ToDoubleFunction<String> encoderPositionNoiseStd,
                                                ToDoubleFunction<String> encoderVelocityNoiseStd,
                                                ToDoubleFunction<String> velocityBreakFrequencyHz,
                                                boolean useDirectVelocityMeasurement)
   {
      Random random = new Random(seed);
      Vector3D[] axes = new Vector3D[numJoints];
      for (int i = 0; i < numJoints; i++)
         axes[i] = new Vector3D(i % 3 == 0 ? 1 : 0, i % 3 == 1 ? 1 : 0, i % 3 == 2 ? 1 : 0);

      RandomFloatingRevoluteJointChain chain = new RandomFloatingRevoluteJointChain(random, axes);
      chain.nextState(random, JointStateType.CONFIGURATION, JointStateType.VELOCITY);
      chain.getElevator().updateFramesRecursively();
      List<RevoluteJoint> joints = chain.getRevoluteJoints();

      List<TestIMU> imus = new ArrayList<>();
      List<IMUSensorReadOnly> imuReadOnly = new ArrayList<>();
      for (int k = 0; k < imuNames.length; k++)
      {
         TestIMU imu = new TestIMU(imuNames[k], joints.get(imuBodyJointIndex[k]).getSuccessor());
         imus.add(imu);
         imuReadOnly.add(imu);
      }
      // Poison an IMU's bias covariance BEFORE the filter is built, so the construction-time guard sees it.
      if (poisonImuIndex >= 0)
         imus.get(poisonImuIndex).setBiasProcessNoiseCovarianceNonFinite();

      TestSensorMap sensorMap = new TestSensorMap(imuReadOnly, joints);

      List<IMUBasedJointStateEstimatorParameters> pairParameters = new ArrayList<>();
      for (int[] pc : pairParentChild)
         pairParameters.add(new IMUBasedJointStateEstimatorParameters("pair", true, imuNames[pc[0]], imuNames[pc[1]], 0.0, 0.0));

      List<RigidBodyBasics> feet = new ArrayList<>();
      int footJoint = footJointIndex >= 0 ? footJointIndex : imuBodyJointIndex[footImuIndex];
      feet.add(joints.get(footJoint).getSuccessor());

      YoRegistry testRegistry = new YoRegistry("test");
      JointLevelKFPreFilter filter = new JointLevelKFPreFilter(sensorMap,
                                                               pairParameters,
                                                               feet,
                                                               useMassMatrixProcessNoise ? chain.getElevator() : null,
                                                               encoderPositionNoiseStd,
                                                               encoderVelocityNoiseStd,
                                                               velocityBreakFrequencyHz,
                                                               useDirectVelocityMeasurement,
                                                               Double.NaN,
                                                               DT,
                                                               testRegistry);
      return new JointLevelKFTestFixture(filter, testRegistry, joints, imus, sensorMap, feet, chain.getElevator(), chain.getRootJoint());
   }

   void setEncoder(OneDoFJointBasics joint, double q)
   {
      sensorMap.setPosition(joint, q);
   }

   /** Firmware-reported joint velocity, i.e. the direct-velocity channel's raw measurement. */
   void setMeasuredVelocity(OneDoFJointBasics joint, double qd)
   {
      sensorMap.setVelocity(joint, qd);
   }

   /**
    * Applies one tick of a self-consistent simulated motion: sets each filtered joint's true {@code q}/{@code q̇}
    * on the live mecano chain and its encoder to the true {@code q}, refreshes the frames, then sets every IMU's
    * gyro from the corresponding link's body-frame angular velocity. Because the pair measurement is
    * {@code ω_child − R·ω_parent}, the (zeroed) base motion cancels and the relative gyro equals {@code J_ang·q̇}
    * at the true state — so the filter can actually observe {@code q̇} (it never reads encoder velocity).
    *
    * @param qTrue  true positions for the filtered joints, in filter state order (length {@code n}).
    * @param qdTrue true velocities for the filtered joints, in filter state order (length {@code n}).
    */
   void applyConsistentMotion(double[] qTrue, double[] qdTrue)
   {
      rootJoint.setJointTwistToZero();      // static base: link gyros come only from the joints (and cancel in pairs)
      for (RevoluteJoint joint : joints)
         joint.setQd(0.0);                  // clear non-path joints so only the filtered joints drive the measurement
      for (int i = 0; i < n; i++)
      {
         OneDoFJointBasics joint = filteredJoints.get(i);
         joint.setQ(qTrue[i]);
         joint.setQd(qdTrue[i]);
         sensorMap.setPosition(joint, qTrue[i]);
      }
      elevator.updateFramesRecursively();
      for (TestIMU imu : imus)
         refreshGyroFromTwist(imu);
   }

   /** Sets an IMU's gyro to its link's body-frame angular velocity (proven recipe from PelvisRotationalStateUpdaterTest). */
   private void refreshGyroFromTwist(TestIMU imu)
   {
      imu.measurementLink.getBodyFixedFrame().getTwistOfFrame(twistTemp);
      twistTemp.changeFrame(imu.getMeasurementFrame());
      Vector3DReadOnly w = twistTemp.getAngularPart();
      imu.setAngularVelocity(w.getX(), w.getY(), w.getZ());
   }

   // ---------------------------------------------------------------------------------------------------------
   // Matrix assertion helpers (the JAX tests lean on jnp.allclose / eigvalsh; these are the EJML equivalents).
   // ---------------------------------------------------------------------------------------------------------

   static void assertAllClose(DMatrixRMaj actual, DMatrixRMaj expected, double tol, String message)
   {
      assertTrue(actual.numRows == expected.numRows && actual.numCols == expected.numCols,
                 message + " — shape mismatch: " + actual.numRows + "x" + actual.numCols + " vs " + expected.numRows + "x" + expected.numCols);
      for (int r = 0; r < actual.numRows; r++)
         for (int c = 0; c < actual.numCols; c++)
            assertTrue(Math.abs(actual.get(r, c) - expected.get(r, c)) <= tol,
                       message + String.format(" — mismatch at (%d,%d): %.6g vs %.6g", r, c, actual.get(r, c), expected.get(r, c)));
   }

   static void assertSymmetric(DMatrixRMaj a, double tol, String message)
   {
      assertTrue(a.numRows == a.numCols, message + " — not square");
      for (int r = 0; r < a.numRows; r++)
         for (int c = r + 1; c < a.numCols; c++)
            assertTrue(Math.abs(a.get(r, c) - a.get(c, r)) <= tol, message + " — asymmetric at (" + r + "," + c + ")");
   }

   static void assertPositiveSemiDefinite(DMatrixRMaj a, String message)
   {
      double[] eig = symmetricEigenvalues(a);
      double max = 0.0;
      double min = Double.POSITIVE_INFINITY;
      for (double e : eig)
      {
         max = Math.max(max, e);
         min = Math.min(min, e);
      }
      assertTrue(min >= -1.0e-6 * Math.max(max, 1.0), message + " — min eigenvalue " + min + " (max " + max + ")");
   }

   static double[] symmetricEigenvalues(DMatrixRMaj a)
   {
      EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(a.numRows, false, true);
      assertTrue(eig.decompose(a.copy()), "eigenvalue decomposition failed");
      double[] values = new double[eig.getNumberOfEigenvalues()];
      for (int i = 0; i < values.length; i++)
      {
         Complex_F64 v = eig.getEigenvalue(i);
         values[i] = v.getReal();
      }
      return values;
   }

   static DMatrixRMaj block(DMatrixRMaj a, int row0, int col0, int rows, int cols)
   {
      DMatrixRMaj out = new DMatrixRMaj(rows, cols);
      for (int r = 0; r < rows; r++)
         for (int c = 0; c < cols; c++)
            out.set(r, c, a.get(row0 + r, col0 + c));
      return out;
   }

   /** A deterministic symmetric positive-definite (size, size) matrix, mirroring the reference's {@code _spd}. */
   static DMatrixRMaj spd(int size, long seed)
   {
      DMatrixRMaj m = new DMatrixRMaj(size, size);
      for (int i = 0; i < size * size; i++)
         m.data[i] = Math.sin(i + 1.0 + seed);
      DMatrixRMaj a = new DMatrixRMaj(size, size);
      CommonOps_DDRM.multTransB(m, m, a); // m mᵀ (PSD)
      for (int i = 0; i < size; i++)
         a.add(i, i, size); // + size·I → PD
      return a;
   }

   static DMatrixRMaj identity(int size)
   {
      DMatrixRMaj i = new DMatrixRMaj(size, size);
      for (int k = 0; k < size; k++)
         i.set(k, k, 1.0);
      return i;
   }

   static DMatrixRMaj scaledIdentity(int size, double value)
   {
      DMatrixRMaj i = new DMatrixRMaj(size, size);
      for (int k = 0; k < size; k++)
         i.set(k, k, value);
      return i;
   }

   static double trace(DMatrixRMaj a)
   {
      double t = 0.0;
      for (int i = 0; i < Math.min(a.numRows, a.numCols); i++)
         t += a.get(i, i);
      return t;
   }

   // ---------------------------------------------------------------------------------------------------------
   // Minimal live sensor implementations.
   // ---------------------------------------------------------------------------------------------------------

   /** IMU with a body-fixed measurement frame, a settable angular-velocity reading, and PD covariances. */
   static final class TestIMU implements IMUSensorReadOnly
   {
      final String name;
      final RigidBodyBasics measurementLink;
      final ReferenceFrame measurementFrame;
      final Vector3D angularVelocity = new Vector3D();
      final Vector3D linearAcceleration = new Vector3D(0.0, 0.0, 9.81);
      final Quaternion orientation = new Quaternion();
      final DMatrixRMaj biasProcessNoiseCovariance = scaledIdentity(3, IMU_BIAS_PROCESS_VAR);
      // Gyro MEASUREMENT noise, kept separate from the bias process noise above so tests can verify the filter
      // reads the right one for the pair-gyro R (regression for the bias-vs-measurement covariance mix-up).
      final DMatrixRMaj angularVelocityNoiseCovariance = scaledIdentity(3, 1.0e-4);
      final DMatrixRMaj genericCovariance = scaledIdentity(3, 1.0e-4);

      TestIMU(String name, RigidBodyBasics measurementLink)
      {
         this.name = name;
         this.measurementLink = measurementLink;
         this.measurementFrame = measurementLink.getBodyFixedFrame();
      }

      void setAngularVelocity(double x, double y, double z)
      {
         angularVelocity.set(x, y, z);
      }

      /** Corrupts this IMU's bias process-noise covariance so it is non-finite (for the NaN-hardening tests). */
      void setBiasProcessNoiseCovarianceNonFinite()
      {
         biasProcessNoiseCovariance.set(0, 0, Double.NaN);
      }

      /** Sets a diagonal (possibly anisotropic) gyro measurement-noise covariance. */
      void setAngularVelocityNoiseCovarianceDiagonal(double xx, double yy, double zz)
      {
         angularVelocityNoiseCovariance.zero();
         angularVelocityNoiseCovariance.set(0, 0, xx);
         angularVelocityNoiseCovariance.set(1, 1, yy);
         angularVelocityNoiseCovariance.set(2, 2, zz);
      }

      /** Sets a diagonal (possibly anisotropic) bias process-noise covariance. */
      void setBiasProcessNoiseCovarianceDiagonal(double xx, double yy, double zz)
      {
         biasProcessNoiseCovariance.zero();
         biasProcessNoiseCovariance.set(0, 0, xx);
         biasProcessNoiseCovariance.set(1, 1, yy);
         biasProcessNoiseCovariance.set(2, 2, zz);
      }

      @Override
      public String getSensorName()
      {
         return name;
      }

      @Override
      public ReferenceFrame getMeasurementFrame()
      {
         return measurementFrame;
      }

      @Override
      public RigidBodyBasics getMeasurementLink()
      {
         return measurementLink;
      }

      @Override
      public QuaternionReadOnly getOrientationMeasurement()
      {
         return orientation;
      }

      @Override
      public Vector3DReadOnly getAngularVelocityMeasurement()
      {
         return angularVelocity;
      }

      @Override
      public Vector3DReadOnly getLinearAccelerationMeasurement()
      {
         return linearAcceleration;
      }

      @Override
      public void getOrientationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
      {
         noiseCovarianceToPack.set(genericCovariance);
      }

      @Override
      public void getAngularVelocityNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
      {
         noiseCovarianceToPack.set(angularVelocityNoiseCovariance);
      }

      @Override
      public void getAngularVelocityBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
      {
         biasProcessNoiseCovarianceToPack.set(biasProcessNoiseCovariance);
      }

      @Override
      public void getLinearAccelerationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
      {
         noiseCovarianceToPack.set(genericCovariance);
      }

      @Override
      public void getLinearAccelerationBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
      {
         biasProcessNoiseCovarianceToPack.set(genericCovariance);
      }
   }

   /** Sensor map returning the fixture IMUs and a settable per-joint encoder state. */
   static final class TestSensorMap implements SensorOutputMapReadOnly
   {
      private final List<IMUSensorReadOnly> imus;
      private final Map<OneDoFJointBasics, SettableJointState> states = new LinkedHashMap<>();
      private final List<OneDoFJointStateReadOnly> stateList = new ArrayList<>();

      TestSensorMap(List<IMUSensorReadOnly> imus, List<? extends OneDoFJointBasics> joints)
      {
         this.imus = imus;
         for (OneDoFJointBasics joint : joints)
         {
            SettableJointState state = new SettableJointState(joint.getName());
            states.put(joint, state);
            stateList.add(state);
         }
      }

      void setPosition(OneDoFJointBasics joint, double q)
      {
         states.get(joint).position = q;
      }

      void setVelocity(OneDoFJointBasics joint, double qd)
      {
         states.get(joint).velocity = qd;
      }

      @Override
      public OneDoFJointStateReadOnly getOneDoFJointOutput(OneDoFJointBasics oneDoFJoint)
      {
         return states.get(oneDoFJoint);
      }

      @Override
      public List<? extends OneDoFJointStateReadOnly> getOneDoFJointOutputs()
      {
         return stateList;
      }

      @Override
      public List<? extends IMUSensorReadOnly> getIMUOutputs()
      {
         return imus;
      }

      @Override
      public ForceSensorDataHolderReadOnly getForceSensorOutputs()
      {
         return null;
      }

      @Override
      public long getWallTime()
      {
         return 0L;
      }

      @Override
      public long getMonotonicTime()
      {
         return 0L;
      }

      @Override
      public long getSyncTimestamp()
      {
         return 0L;
      }
   }

   static final class SettableJointState implements OneDoFJointStateReadOnly
   {
      private final String name;
      double position;
      double velocity;
      double effort;

      SettableJointState(String name)
      {
         this.name = name;
      }

      @Override
      public String getJointName()
      {
         return name;
      }

      @Override
      public double getPosition()
      {
         return position;
      }

      @Override
      public double getVelocity()
      {
         return velocity;
      }

      @Override
      public double getAcceleration()
      {
         return 0.0;
      }

      @Override
      public double getEffort()
      {
         return effort;
      }

      @Override
      public boolean isJointEnabled()
      {
         return true;
      }
   }
}
