package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.lang.management.ManagementFactory;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.lieGroup.SO3LieGroupTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Regression guard: the InEKF predict/update hot path and the SE_k(3)/SO(3) Lie-group helpers it calls
 * every tick must not allocate on the estimator thread.
 *
 * <p>Rather than the instrumenter-based {@code AllocationProfiler} (which needs a javaagent and a heavy
 * simulation), this reads the per-thread allocation counter directly via {@link com.sun.management.ThreadMXBean}.
 * After a JIT warm-up, a truly allocation-free loop leaves the counter unchanged; a single small object per
 * tick (a {@code Quaternion}, {@code Vector3D}, a boxed {@code Double}, …) shows up as tens to hundreds of
 * bytes per tick, far above {@link #MAX_BYTES_PER_TICK}. Before the pre-allocation fixes this path allocated
 * ~0.9 kB/tick (Lie-group temporaries in {@code SEK3_Utils}/{@code SO3LieGroupTools} plus contact-provider
 * autoboxing); after them it is 0.</p>
 */
public class InvariantEstimatorAllocationTest
{
   private static final int NUMBER_OF_CONTACTS = 2;
   private static final double DT = 0.001;

   /** Allowed slack, in bytes per tick, above which the path is considered to be generating garbage. */
   private static final double MAX_BYTES_PER_TICK = 16.0;

   private static final int WARMUP_TICKS = 50_000;
   private static final int MEASURED_TICKS = 200_000;

   private final com.sun.management.ThreadMXBean threadMXBean =
         (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();

   /**
    * Drives {@link InvariantEKF#predict} + two {@link InvariantEKF#update}s per tick (exactly what the main
    * estimator does each control tick) and asserts the estimator thread allocates nothing.
    */
   @Test
   public void testPredictUpdateHotPathIsAllocationFree()
   {
      assumeTrue(threadMXBean.isThreadAllocatedMemorySupported(), "Thread allocation counting not supported on this JVM.");
      threadMXBean.setThreadAllocatedMemoryEnabled(true);

      InvariantEKF ekf = new InvariantEKF(NUMBER_OF_CONTACTS, 1.0e-4, 1.0e-3, 1.0e-6);

      RotationMatrix initialRotation = new RotationMatrix();
      Vector3D initialVelocity = new Vector3D();
      Point3D initialPosition = new Point3D();
      Tuple3DReadOnly[] contactPositions = {new Point3D(0.1, 0.1, 0.0), new Point3D(0.1, -0.1, 0.0)};

      int m = 9 + 3 * NUMBER_OF_CONTACTS;
      DMatrixRMaj initialCovariance = new DMatrixRMaj(m, m);
      for (int i = 0; i < m; i++)
         initialCovariance.set(i, i, 1.0);
      ekf.initialize(initialRotation, initialVelocity, initialPosition, contactPositions, initialCovariance);

      Vector3D angularVelocity = new Vector3D(0.01, -0.02, 0.03);
      Vector3D linearAcceleration = new Vector3D(0.0, 0.0, 9.81);
      Vector3D contactMeasurement = new Vector3D(0.0, 0.12, -0.90);
      Matrix3D measurementCovariance = new Matrix3D();
      measurementCovariance.set(1.0e-4, 0.0, 0.0,
                                0.0, 1.0e-4, 0.0,
                                0.0, 0.0, 1.0e-4);

      Runnable oneTick = () ->
      {
         // Mirror the per-tick slip-variance retune the estimator does, then predict + update each foot.
         ekf.setContactSlipVariance(0, 1.0e-6);
         ekf.setContactSlipVariance(1, 1.0e-6);
         ekf.predict(angularVelocity, linearAcceleration, DT);
         ekf.update(0, contactMeasurement, measurementCovariance);
         ekf.update(1, contactMeasurement, measurementCovariance);
      };

      assertAllocationFree("InEKF predict + 2×update", oneTick);
   }

   /**
    * Exercises the allocation-free SE_k(3) overloads and {@link SO3LieGroupTools#exp} directly, so a
    * regression is localized to the Lie-group helpers rather than the whole filter.
    */
   @Test
   public void testLieGroupScratchOverloadsAreAllocationFree()
   {
      assumeTrue(threadMXBean.isThreadAllocatedMemorySupported(), "Thread allocation counting not supported on this JVM.");
      threadMXBean.setThreadAllocatedMemoryEnabled(true);

      double[] xi = {0.03, -0.02, 0.05, 0.10, -0.20, 0.30, -0.40, 0.50, 0.05};
      double[] recovered = new double[xi.length];
      DMatrixRMaj groupElement = new DMatrixRMaj(5, 5);
      DMatrixRMaj adjoint = new DMatrixRMaj(15, 15);

      Vector3D vectorScratch = new Vector3D();
      RotationMatrix rotationScratch = new RotationMatrix();
      Matrix3D matrixScratch = new Matrix3D();

      Vector3D omega = new Vector3D(0.01, 0.02, -0.03);
      RotationMatrix rotationIncrement = new RotationMatrix();

      Runnable oneTick = () ->
      {
         SEK3Utils.exp(xi, groupElement, vectorScratch, rotationScratch, matrixScratch);
         SEK3Utils.adjoint(groupElement, adjoint, rotationScratch, vectorScratch, matrixScratch);
         SEK3Utils.log(groupElement, recovered, rotationScratch, vectorScratch, matrixScratch);
         SO3LieGroupTools.exp(omega, rotationIncrement);
      };

      assertAllocationFree("SEK3_Utils.exp/adjoint/log + SO3LieGroupTools.exp", oneTick);
   }

   /** Warms the JIT, then asserts the estimator thread's allocated bytes stay flat across {@link #MEASURED_TICKS}. */
   private void assertAllocationFree(String description, Runnable oneTick)
   {
      for (int i = 0; i < WARMUP_TICKS; i++)
         oneTick.run();

      long threadId = Thread.currentThread().getId();
      long before = threadMXBean.getThreadAllocatedBytes(threadId);
      for (int i = 0; i < MEASURED_TICKS; i++)
         oneTick.run();
      long after = threadMXBean.getThreadAllocatedBytes(threadId);

      double bytesPerTick = (after - before) / (double) MEASURED_TICKS;
      assertTrue(bytesPerTick < MAX_BYTES_PER_TICK,
                 String.format("%s allocated %.2f bytes/tick (%d bytes over %d ticks); expected < %.1f. "
                               + "Something on the hot path is creating garbage every tick.",
                               description, bytesPerTick, after - before, MEASURED_TICKS, MAX_BYTES_PER_TICK));
   }
}
