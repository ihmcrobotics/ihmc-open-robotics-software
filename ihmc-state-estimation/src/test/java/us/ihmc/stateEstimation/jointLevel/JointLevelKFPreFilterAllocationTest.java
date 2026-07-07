package us.ihmc.stateEstimation.jointLevel;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.lang.management.ManagementFactory;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.OneDoFJointStateReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.IMUBasedJointStateEstimatorParameters;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Guards the {@link JointLevelKFPreFilter} hot path (phase-1 joint update + phase-2 stance anchor) against
 * per-tick allocation on the estimator thread.
 *
 * <p>Motivation: on hardware the alpha pre-filter ran fine but {@code JOINT_KF} threw an EtherCAT error every
 * control tick. The cause was garbage on the real-time estimator thread, from two sources the alpha filter
 * never had (which is why only {@code JOINT_KF} failed):</p>
 * <ol>
 * <li>the encoder measurement update inverts an n&times;n innovation matrix (n = filtered joints), and
 * {@code CommonOps_DDRM.invert} news up a fresh LU decomposition + solver on every call once the matrix is
 * wider than 5&times;5 (~0.8 kB/tick). Fixed by reusing a pre-warmed {@code LinearSolverDense}.</li>
 * <li>a redundant per-joint {@code j.updateFrame()} in each measurement update, which allocates inside
 * {@code MovingReferenceFrame.update()} (~1 kB/tick). Removed — the estimator already keeps all frames
 * current via {@code updateFramesRecursively()} each tick.</li>
 * </ol>
 * <p>Together they were ~1.8 kB/tick. This test drives the whole hot path and locks in 0.</p>
 *
 * <p>Measurement mirrors {@code InvariantEstimatorAllocationTest}: read the estimator thread's own allocation
 * counter via {@link com.sun.management.ThreadMXBean} after a JIT warm-up. A truly allocation-free loop leaves
 * the counter flat.</p>
 */
public class JointLevelKFPreFilterAllocationTest
{
   private static final double DT = 1.0e-3;
   /** Allowed slack in bytes per tick; ~20x below the ~600 B/tick the old invert() path produced. */
   private static final double MAX_BYTES_PER_TICK = 32.0;
   private static final int WARMUP_TICKS = 20_000;
   private static final int MEASURED_TICKS = 60_000;

   private final com.sun.management.ThreadMXBean threadMXBean =
         (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();

   @Test
   public void testPhase1AndPhase2HotPathIsAllocationFree()
   {
      assumeTrue(threadMXBean.isThreadAllocatedMemorySupported(), "Thread allocation counting not supported on this JVM.");
      threadMXBean.setThreadAllocatedMemoryEnabled(true);

      Fixture fixture = new Fixture(new Random(9001L));
      JointLevelKFPreFilter filter = fixture.filter;
      List<RigidBodyBasics> trustedFeet = fixture.feet;

      filter.initialize();

      // A single control tick: phase 1 (predict + encoder + pair-gyro updates) then phase 2 (stance anchor).
      Runnable oneTick = () ->
      {
         filter.computeJointState();
         filter.computeImuBiases(trustedFeet);
      };

      for (int i = 0; i < WARMUP_TICKS; i++)
         oneTick.run();

      long threadId = Thread.currentThread().getId();
      long before = threadMXBean.getThreadAllocatedBytes(threadId);
      for (int i = 0; i < MEASURED_TICKS; i++)
         oneTick.run();
      long after = threadMXBean.getThreadAllocatedBytes(threadId);

      double bytesPerTick = (after - before) / (double) MEASURED_TICKS;
      assertTrue(bytesPerTick < MAX_BYTES_PER_TICK,
                 String.format("JointLevelKFPreFilter hot path allocated %.2f bytes/tick (%d bytes over %d ticks); "
                               + "expected < %.1f. Something on the estimator thread is creating garbage every tick.",
                               bytesPerTick, after - before, MEASURED_TICKS, MAX_BYTES_PER_TICK));
   }

   /** Sanity: the filter constructs, initializes, ticks, and produces finite estimates (no NaN / no per-tick throw). */
   @Test
   public void testHotPathStaysFinite()
   {
      Fixture fixture = new Fixture(new Random(9002L));
      JointLevelKFPreFilter filter = fixture.filter;

      filter.initialize();
      for (int i = 0; i < 1_000; i++)
      {
         filter.computeJointState();
         filter.computeImuBiases(fixture.feet);
      }

      for (OneDoFJointBasics joint : fixture.filteredJoints)
      {
         assertTrue(Double.isFinite(filter.getEstimatedJointPosition(joint)), "Non-finite position estimate.");
         assertTrue(Double.isFinite(filter.getEstimatedJointVelocity(joint)), "Non-finite velocity estimate.");
      }

      FrameVector3DReadOnly baseBias = filter.getAngularVelocityBiasInIMUFrame(fixture.baseIMU);
      assertTrue(Double.isFinite(baseBias.getX()) && Double.isFinite(baseBias.getY()) && Double.isFinite(baseBias.getZ()),
                 "Non-finite base IMU bias estimate.");
   }

   /**
    * A synthetic IMU-pair setup: a 10-joint floating chain with two IMUs placed far apart so their kinematic
    * chain spans 8 revolute joints (n = 8 &gt; 5, which forces the allocating LU branch in the old code).
    */
   private static final class Fixture
   {
      private final JointLevelKFPreFilter filter;
      private final List<RigidBodyBasics> feet;
      private final List<OneDoFJointBasics> filteredJoints = new ArrayList<>();
      private final IMUSensorReadOnly baseIMU;

      private Fixture(Random random)
      {
         Vector3D[] jointAxes = new Vector3D[10];
         for (int i = 0; i < jointAxes.length; i++)
            jointAxes[i] = new Vector3D(i % 3 == 0 ? 1 : 0, i % 3 == 1 ? 1 : 0, i % 3 == 2 ? 1 : 0);

         RandomFloatingRevoluteJointChain chain = new RandomFloatingRevoluteJointChain(random, jointAxes);
         chain.nextState(random, JointStateType.CONFIGURATION, JointStateType.VELOCITY);
         chain.getElevator().updateFramesRecursively();
         List<RevoluteJoint> joints = chain.getRevoluteJoints();

         RigidBodyBasics parentLink = joints.get(1).getSuccessor();
         RigidBodyBasics childLink = joints.get(9).getSuccessor();

         TestIMU parentIMU = new TestIMU("parentIMU", parentLink);
         TestIMU childIMU = new TestIMU("childIMU", childLink);
         this.baseIMU = parentIMU;

         List<IMUSensorReadOnly> imus = new ArrayList<>();
         imus.add(parentIMU);
         imus.add(childIMU);

         TestSensorMap sensorMap = new TestSensorMap(imus, joints);

         List<IMUBasedJointStateEstimatorParameters> pairParameters = new ArrayList<>();
         pairParameters.add(new IMUBasedJointStateEstimatorParameters("testPair", true, "parentIMU", "childIMU", 0.0, 0.0));

         this.feet = new ArrayList<>();
         feet.add(childLink); // base(parentLink) -> foot(childLink) leg chain is fully in state -> anchor usable

         this.filter = new JointLevelKFPreFilter(sensorMap, pairParameters, feet, DT, new YoRegistry("test"));

         for (OneDoFJointBasics joint : joints)
            if (filter.containsJoint(joint))
               filteredJoints.add(joint);
      }
   }

   /** Minimal live IMU: fixed measurement, body-fixed measurement frame, positive-definite covariances. */
   private static final class TestIMU implements IMUSensorReadOnly
   {
      private final String name;
      private final RigidBodyBasics measurementLink;
      private final ReferenceFrame measurementFrame;
      private final Vector3D angularVelocity = new Vector3D(0.01, -0.02, 0.03);
      private final Vector3D linearAcceleration = new Vector3D(0.0, 0.0, 9.81);
      private final Quaternion orientation = new Quaternion();
      private final DMatrixRMaj covariance = diagonal3x3(1.0e-4);

      private TestIMU(String name, RigidBodyBasics measurementLink)
      {
         this.name = name;
         this.measurementLink = measurementLink;
         this.measurementFrame = measurementLink.getBodyFixedFrame();
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
         noiseCovarianceToPack.set(covariance);
      }

      @Override
      public void getAngularVelocityNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
      {
         noiseCovarianceToPack.set(covariance);
      }

      @Override
      public void getAngularVelocityBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
      {
         biasProcessNoiseCovarianceToPack.set(covariance);
      }

      @Override
      public void getLinearAccelerationNoiseCovariance(DMatrixRMaj noiseCovarianceToPack)
      {
         noiseCovarianceToPack.set(covariance);
      }

      @Override
      public void getLinearAccelerationBiasProcessNoiseCovariance(DMatrixRMaj biasProcessNoiseCovarianceToPack)
      {
         biasProcessNoiseCovarianceToPack.set(covariance);
      }
   }

   /** Minimal sensor map: returns the two IMUs and a cached dummy state per joint (no per-lookup allocation). */
   private static final class TestSensorMap implements SensorOutputMapReadOnly
   {
      private final List<IMUSensorReadOnly> imus;
      private final Map<OneDoFJointBasics, OneDoFJointStateReadOnly> jointOutputs = new HashMap<>();
      private final List<OneDoFJointStateReadOnly> jointOutputList = new ArrayList<>();

      private TestSensorMap(List<IMUSensorReadOnly> imus, List<? extends OneDoFJointBasics> joints)
      {
         this.imus = imus;
         for (OneDoFJointBasics joint : joints)
         {
            OneDoFJointStateReadOnly state = OneDoFJointStateReadOnly.dummyOneDoFJointState(joint.getName());
            jointOutputs.put(joint, state);
            jointOutputList.add(state);
         }
      }

      @Override
      public OneDoFJointStateReadOnly getOneDoFJointOutput(OneDoFJointBasics oneDoFJoint)
      {
         return jointOutputs.get(oneDoFJoint);
      }

      @Override
      public List<? extends OneDoFJointStateReadOnly> getOneDoFJointOutputs()
      {
         return jointOutputList;
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

   private static DMatrixRMaj diagonal3x3(double value)
   {
      DMatrixRMaj matrix = new DMatrixRMaj(3, 3);
      for (int i = 0; i < 3; i++)
         matrix.set(i, i, value);
      return matrix;
   }
}
