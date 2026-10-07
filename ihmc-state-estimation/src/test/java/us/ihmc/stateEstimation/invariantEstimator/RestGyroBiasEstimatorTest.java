package us.ihmc.stateEstimation.invariantEstimator;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RestGyroBiasEstimatorTest
{
   private static final double DT = 0.001;

   /** A slow lean while standing: the raw mean is bias plus lean rate, and the kinematic rotation removes the lean. */
   @Test
   public void testRecoversTheBiasUnderASlowLean()
   {
      Random random = new Random(1006L);
      Vector3D trueBias = new Vector3D(0.008, -0.002, 0.001);
      Vector3D leanRate = new Vector3D(0.0, 0.01, 0.0); // 10 mrad/s about IMU Y: 3x the bias, would dominate a plain mean
      RestGyroBiasEstimator estimator = new RestGyroBiasEstimator(2.0, 0.1, new YoRegistry("test"));

      Quaternion imuInSole = new Quaternion(0.1, -0.05, 1.2);
      Vector3D gyro = new Vector3D();
      Quaternion step = new Quaternion();
      for (int tick = 0; tick < 2000; tick++)
      {
         gyro.set(leanRate);
         gyro.add(trueBias);
         gyro.add(0.002 * random.nextGaussian(), 0.002 * random.nextGaussian(), 0.002 * random.nextGaussian());
         estimator.update(gyro, true, imuInSole, imuInSole, DT);
         step.setRotationVector(leanRate.getX() * DT, leanRate.getY() * DT, leanRate.getZ() * DT);
         imuInSole.multiply(step); // body-frame rotation: R_now = R_prev * exp(omega dt)
      }

      assertEquals(1, estimator.getNumberOfUpdates());
      assertEquals(trueBias.getX(), estimator.getBias().getX(), 3.0e-4);
      assertEquals(trueBias.getY(), estimator.getBias().getY(), 3.0e-4);
      assertEquals(trueBias.getZ(), estimator.getBias().getZ(), 3.0e-4);
   }

   @Test
   public void testZeroUntilAWindowOfRestAndHeldThroughWalking()
   {
      Vector3D bias = new Vector3D(0.005, 0.0, 0.0);
      Quaternion fixed = new Quaternion();
      RestGyroBiasEstimator estimator = new RestGyroBiasEstimator(1.0, 0.1, null);

      for (int tick = 0; tick < 999; tick++)
         estimator.update(bias, true, fixed, fixed, DT);
      assertEquals(0.0, estimator.getBias().norm(), "nothing published before a full window");

      estimator.update(bias, false, fixed, fixed, DT); // a foot lifts: the rest ends unpublished
      for (int tick = 0; tick < 999; tick++)
         estimator.update(bias, true, fixed, fixed, DT);
      assertEquals(0.0, estimator.getBias().norm(), "the interrupted rest must not count toward the window");

      estimator.update(bias, true, fixed, fixed, DT);
      assertEquals(0.005, estimator.getBias().getX(), 1.0e-12);

      Vector3D swing = new Vector3D(0.5, 0.2, 0.0);
      for (int tick = 0; tick < 500; tick++)
         estimator.update(swing, false, fixed, fixed, DT);
      assertEquals(0.005, estimator.getBias().getX(), 1.0e-12, "held while walking");
   }

   @Test
   public void testFastMotionWithBothFeetDownIsNotARest()
   {
      Quaternion fixed = new Quaternion();
      RestGyroBiasEstimator estimator = new RestGyroBiasEstimator(0.5, 0.1, null);
      Vector3D fast = new Vector3D(0.0, 0.2, 0.0);
      for (int tick = 0; tick < 2000; tick++)
         estimator.update(fast, true, fixed, fixed, DT);
      assertEquals(0, estimator.getNumberOfUpdates());
   }
}
