package us.ihmc.kalman;

import org.ejml.data.DenseMatrix64F;

/**
 * adapted from http://code.google.com/p/efficient-java-matrix-library/wiki/KalmanFilterExamples
 */
public interface KalmanFilter
{
   public abstract void configure(DenseMatrix64F F, DenseMatrix64F G, DenseMatrix64F H);

   public abstract void setProcessNoiseCovariance(DenseMatrix64F Q);
   
   public abstract void setMeasurementNoiseCovariance(DenseMatrix64F R);
   
   public abstract void setState(DenseMatrix64F x, DenseMatrix64F P);

   public abstract void predict(DenseMatrix64F u);

   public abstract void update(DenseMatrix64F y);

   public abstract DenseMatrix64F getState();

   public abstract DenseMatrix64F getCovariance();
}
