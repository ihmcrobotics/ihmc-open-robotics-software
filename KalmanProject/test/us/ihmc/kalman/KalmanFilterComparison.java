package us.ihmc.kalman;

import junit.framework.TestCase;
import us.ihmc.kalman.imu.QuaternionBasedArrayFullIMUKalmanFilter;
import us.ihmc.kalman.imu.QuaternionBasedJamaFullIMUKalmanFilter;

public class KalmanFilterComparison extends TestCase
{
   @SuppressWarnings("unused")
   private static final double DT = 0.001;

   public KalmanFilterComparison(String name)
   {
      super(name);
   }

   protected void setUp() throws Exception
   {
      super.setUp();
   }

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
//
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



   protected void tearDown() throws Exception
   {
      super.tearDown();
   }

}
