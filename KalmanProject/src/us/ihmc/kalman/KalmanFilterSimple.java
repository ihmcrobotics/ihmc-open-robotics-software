package us.ihmc.kalman;

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;

/**
 * Adapted from http://code.google.com/p/efficient-java-matrix-library/wiki/KalmanFilterExamples
 *
 * A Kalman filter implemented using SimpleMatrix.  The code tends to be easier to
 * read and write, but the performance is degraded due to excessive creation/destruction of
 * memory and the use of more generic algorithms.  This also demonstrates how code can be
 * seamlessly implemented using both SimpleMatrix and DenseMatrix64F.  This allows code
 * to be quickly prototyped or to be writen either by novices or experts.
 *
 * @author Peter Abeles
 */
public class KalmanFilterSimple implements KalmanFilter
{
   // system model
   private SimpleMatrix F;
   private SimpleMatrix G;
   private SimpleMatrix H;

   // noise model
   private SimpleMatrix Q;
   private SimpleMatrix R;

   // sytem state estimate
   private SimpleMatrix x;
   private SimpleMatrix P;

   public void configure(DenseMatrix64F F, DenseMatrix64F G, DenseMatrix64F H)
   {
      this.F = new SimpleMatrix(F);
      this.G = new SimpleMatrix(G);
      this.H = new SimpleMatrix(H);
   }

   public void setProcessNoiseCovariance(DenseMatrix64F Q)
   {
      this.Q = new SimpleMatrix(Q);
   }

   public void setMeasurementNoiseCovariance(DenseMatrix64F R)
   {
      this.R = new SimpleMatrix(R);
   }

   public void setState(DenseMatrix64F x, DenseMatrix64F P)
   {
      this.x = new SimpleMatrix(x);
      this.P = new SimpleMatrix(P);
   }

   public void predict(DenseMatrix64F uDense)
   {
      // x = F x + G u
      SimpleMatrix u = SimpleMatrix.wrap(uDense);
      x = (F.mult(x)).plus((G.mult(u)));

      // P = F P F' + Q
      P = F.mult(P).mult(F.transpose()).plus(Q);
   }

   public void update(DenseMatrix64F yDense)
   {
      // a fast way to make the matrices usable by SimpleMatrix
      SimpleMatrix y = SimpleMatrix.wrap(yDense);

      // z = y - H x
      SimpleMatrix z = y.minus(H.mult(x));

      // S = H P H' + R
      SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);

      // K = PH'S^(-1)
      SimpleMatrix K = P.mult(H.transpose().mult(S.invert()));

      // x = x + Kz
      x = x.plus(K.mult(z));

      // P = (I-kH)P = P - KHP
      P = P.minus(K.mult(H).mult(P));
   }

   public DenseMatrix64F getState()
   {
      return x.getMatrix();
   }

   public DenseMatrix64F getCovariance()
   {
      return P.getMatrix();
   }
}
