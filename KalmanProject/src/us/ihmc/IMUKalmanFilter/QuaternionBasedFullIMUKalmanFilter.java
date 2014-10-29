package us.ihmc.IMUKalmanFilter;

import Jama.Matrix;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2004</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public interface QuaternionBasedFullIMUKalmanFilter
{
   /**
    * initialize
    *
    * @param accel Matrix
    * @param pqr Matrix
    * @param d double
    */
   public void initialize(Matrix accel, Matrix pqr, double d);

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
   public void imuUpdate(Matrix pqr);

   /**
    * compassUpdate
    *
    * @param d double
    * @param accel Matrix
    */
   public void compassUpdate(double d, Matrix accel);

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
    * @return Matrix
    */
   public void getQuaternion(Matrix quaternionMatrix);

   /**
    * accel2quaternions
    *
    * @param accel Matrix
    * @param d double
    * @return double[]
    */
   public void accel2quaternions(Matrix accel, double d, double[] quaternions);

}
