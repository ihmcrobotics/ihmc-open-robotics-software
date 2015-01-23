package us.ihmc.imageProcessing.segmentation;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CovarianceOps;

/**
 * Gaussian distribution in 3D
 *
 * @author Peter Abeles
 */
public class Gaussian3D_F64
{
   public double[] mean = new double[3];
   public DenseMatrix64F covariance = new DenseMatrix64F(3,3);
   public DenseMatrix64F covarianceInv = new DenseMatrix64F(3,3);

   public void setMean(double x0, double x1, double x2) {
      mean[0] = x0;
      mean[1] = x1;
      mean[2] = x2;
   }

   public void computeInverse() {
      CovarianceOps.invert(covariance,covarianceInv);
   }

   public double[] getMean()
   {
      return mean;
   }

   public void setMean(double[] mean)
   {
      this.mean = mean;
   }

   public DenseMatrix64F getCovariance()
   {
      return covariance;
   }

   public void setCovariance(DenseMatrix64F covariance)
   {
      this.covariance = covariance;
   }

   public DenseMatrix64F getCovarianceInv()
   {
      return covarianceInv;
   }

   public void setCovarianceInv(DenseMatrix64F covarianceInv)
   {
      this.covarianceInv = covarianceInv;
   }

   public void print() {
      System.out.printf("Mean = [%6.2f %6.2f %6.2f]\n",mean[0],mean[1],mean[2]);
      System.out.println("Covariance:");
      covariance.print();
   }
}
