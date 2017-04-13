package us.ihmc.kalman.imu;

import org.ejml.data.DenseMatrix64F;

public interface QuaternionBasedFullIMUKalmanFilter
{
   /**
    * initialize
    *
    * @param accel DenseMatrix64F
    * @param pqr DenseMatrix64F
    * @param d double
    */
   public void initialize(DenseMatrix64F accel, DenseMatrix64F pqr, double d);

   /**
    * reset
    */
   public void reset();

   /**
    * setNoiseParameters
    *
    * @param d double
    * @param d1 double
    */
   public void setNoiseParameters(double d, double d1);

   /**
    * imuUpdate
    *
    * @param pqr Matrix
    */
   public void imuUpdate(DenseMatrix64F pqr);

   /**
    * compassUpdate
    *
    * @param d double
    * @param accel DenseMatrix64F
    */
   public void compassUpdate(double d, DenseMatrix64F accel);

   /**
    * getBias
    *
    * @param i int
    * @return double
    */
   public double getBias(int i);

   /**
    * getQuaternion
    *
    * @return DenseMatrix64F
    */
   public void getQuaternion(DenseMatrix64F quaternionMatrix);

   /**
    * accel2quaternions
    *
    * @param accel Matrix
    * @param d double
    * @return double[]
    */
   public void accel2quaternions(DenseMatrix64F accel, double d, double[] quaternions);

}
