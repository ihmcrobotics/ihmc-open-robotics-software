package us.ihmc.kalman.imu.testCases;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.kalman.imu.QuaternionBasedArrayFullIMUKalmanFilter;
import us.ihmc.kalman.imu.QuaternionBasedJamaFullIMUKalmanFilter;

public class KalmanFilterComparisonTest
{
   @SuppressWarnings("unused")
   private static final double DT = 0.001;

   @Before
   public void setUp() throws Exception
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRepeatability()
   {
      @SuppressWarnings("unused") QuaternionBasedJamaFullIMUKalmanFilter quaternionBasedFullIMUKalmanFilter = null;
      @SuppressWarnings("unused") QuaternionBasedArrayFullIMUKalmanFilter fastQuaternionBasedFullIMUKalmanFilter = null;
   }

// private QuaternionBasedFullIMUKalmanFilter createAndInitSlowKalman()
// {
//    QuaternionBasedFullIMUKalmanFilter ret = new QuaternionBasedFullIMUKalmanFilter(DT);
//
//    ret.initialize(accel, pqr, robot.yaw.val);
//    ret.reset();
//    return ret;
// }
   
// private FastQuaternionBasedFullIMUKalmanFilter createAndInitFastKalman()
//{
// FastQuaternionBasedFullIMUKalmanFilter ret = new FastQuaternionBasedFullIMUKalmanFilter(DT);
//    double[][] accel = new double[3][1];
//    double[][] pqr = new double[3][1];
//    accel[0][0] = -x_accel.val;
//    accel[1][0] = -y_accel.val;
//    accel[2][0] = -z_accel.val;
//
//    pqr[0][0] = 0.0;
//    pqr[1][0] = 0.0;
//    pqr[2][0] = 0.0;
//    fastFullIMUKalmanFilter.initialize(accel, pqr, robot.yaw.val);
//    fastFullIMUKalmanFilter.reset(fastFullIMUKalmanFilter.P);
//
// return ret;
//}

   protected void compareFilters(QuaternionBasedJamaFullIMUKalmanFilter filterOne, QuaternionBasedJamaFullIMUKalmanFilter filterTwo)
   {
   }

   @After
   public void tearDown() throws Exception
   {
   }
}
